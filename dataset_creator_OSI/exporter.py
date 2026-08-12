"""MCAP exporter for OSI trace data with optional raw ROS message embedding.

Uses the asam-osi-utilities MCAPChannel helper with a raw mcap.writer.Writer
to support both OSI and non-OSI channels in a single spec-compliant MCAP file.

Compression: zstd level 19 (monkey-patched), 32 MiB chunks.
"""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import TYPE_CHECKING

import mcap.writer as mcap_writer_module
from google.protobuf.message import Message
from mcap.writer import CompressionType, Writer as McapRawWriter

from osi_utilities.tracefile.mcap_channel import MCAPChannel
from osi_utilities.tracefile.writers.multi import prepare_required_file_metadata

if TYPE_CHECKING:
    from typing import IO

logger = logging.getLogger(__name__)

ZSTD_LEVEL = 19
CHUNK_SIZE = 32 * 1024 * 1024  # 32 MiB


_zstd_patch_refcount = 0
_zstd_original: object = None


def _patch_zstd_level() -> None:
    """Monkey-patch mcap's zstd.compress to use level 19.

    Uses reference counting so multiple exporters share one patch safely.
    """
    global _zstd_patch_refcount, _zstd_original

    if _zstd_patch_refcount == 0:
        import zstandard

        _zstd_original = mcap_writer_module.zstandard.compress  # type: ignore[attr-defined]
        compressor = zstandard.ZstdCompressor(level=ZSTD_LEVEL)

        def _compress_high(data: bytes) -> bytes:
            return compressor.compress(data)

        mcap_writer_module.zstandard.compress = _compress_high  # type: ignore[attr-defined]

    _zstd_patch_refcount += 1


def _unpatch_zstd_level() -> None:
    """Restore the original zstd compress function when last exporter closes."""
    global _zstd_patch_refcount, _zstd_original

    if _zstd_patch_refcount > 0:
        _zstd_patch_refcount -= 1

    if _zstd_patch_refcount == 0 and _zstd_original is not None:
        mcap_writer_module.zstandard.compress = _zstd_original  # type: ignore[attr-defined]
        _zstd_original = None


class MCAPExporter:
    """Writes multi-channel OSI trace data to a single MCAP file.

    Uses MCAPChannel for OSI channels and direct mcap.Writer access
    for optional non-OSI (raw ROS) channels.

    Args:
        output_path: Path to the output .mcap file.
        include_raw: If True, also embed original ROS messages as non-OSI channels.
        description: Optional description for the MCAP file metadata.
    """

    def __init__(
        self,
        output_path: str | Path,
        include_raw: bool = False,
        description: str = "",
    ) -> None:
        self._output_path = Path(output_path)
        self._include_raw = include_raw
        self._description = description

        self._file: IO[bytes] | None = None
        self._writer: McapRawWriter | None = None
        self._osi_channel: MCAPChannel | None = None
        self._raw_channels: dict[str, int] = {}  # topic -> channel_id for ROS
        self._written_count = 0

    def open(self) -> None:
        """Open the MCAP file and initialize the writer."""
        if self._writer is not None:
            raise RuntimeError("Exporter already open — call close() first")

        _patch_zstd_level()

        self._file = open(self._output_path, "wb")  # noqa: SIM115
        self._writer = McapRawWriter(
            self._file,
            chunk_size=CHUNK_SIZE,
            compression=CompressionType.ZSTD,
        )
        self._writer.start(library="osi-utilities-python")

        metadata = prepare_required_file_metadata()
        metadata["creation_time"] = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        if self._description:
            metadata["description"] = self._description
        self._writer.add_metadata(name="net.asam.osi.trace", data=metadata)

        self._osi_channel = MCAPChannel(self._writer)
        self._written_count = 0

        logger.info("Opened MCAP file: %s (zstd-%d, %d MiB chunks)",
                     self._output_path, ZSTD_LEVEL, CHUNK_SIZE // (1024 * 1024))

    def add_osi_channel(
        self,
        topic: str,
        message_class: type[Message],
        metadata: dict[str, str] | None = None,
    ) -> int:
        """Register an OSI channel (protobuf schema, OSI-compliant metadata).

        Args:
            topic: Channel topic name (e.g. sensor name from config).
            message_class: The protobuf message class (e.g. SensorData).
            metadata: Optional additional channel metadata.

        Returns:
            The channel ID.
        """
        if self._osi_channel is None:
            raise RuntimeError("Exporter not open")
        return self._osi_channel.add_channel(topic, message_class, metadata)

    def add_raw_channel(
        self,
        topic: str,
        ros_msg_type: str,
    ) -> int:
        """Register a non-OSI channel for raw ROS messages.

        Args:
            topic: Channel topic name (e.g. the ROS topic).
            ros_msg_type: ROS message type string (e.g. "sensor_msgs/PointCloud2").

        Returns:
            The channel ID.
        """
        if self._writer is None:
            raise RuntimeError("Exporter not open")
        if topic in self._raw_channels:
            raise RuntimeError(f"Raw channel '{topic}' already registered")

        # Use schema_id=0 (no schema) for raw pass-through channels per MCAP spec
        channel_id = self._writer.register_channel(
            topic=f"raw/{topic}",
            message_encoding="ros1msg",
            schema_id=0,
        )
        self._raw_channels[topic] = channel_id
        return channel_id

    def write_osi_message(self, topic: str, message: Message) -> bool:
        """Write an OSI protobuf message to a registered OSI channel.

        Args:
            topic: The channel topic.
            message: The protobuf message.

        Returns:
            True on success.
        """
        if self._osi_channel is None:
            logger.warning("Cannot write OSI message for '%s': exporter not open", topic)
            return False
        ok = self._osi_channel.write_message(message, topic)
        if ok:
            self._written_count += 1
        return ok

    def write_raw_message(
        self,
        topic: str,
        data: bytes,
        timestamp_ns: int,
    ) -> bool:
        """Write a raw ROS message to a registered non-OSI channel.

        Args:
            topic: The original ROS topic.
            data: Serialized ROS message bytes.
            timestamp_ns: Timestamp in nanoseconds.

        Returns:
            True on success.
        """
        if self._writer is None:
            logger.warning("Cannot write raw message for '%s': exporter not open", topic)
            return False
        if topic not in self._raw_channels:
            logger.warning("Cannot write raw message: channel '%s' not registered", topic)
            return False
        try:
            self._writer.add_message(
                channel_id=self._raw_channels[topic],
                log_time=timestamp_ns,
                data=data,
                publish_time=timestamp_ns,
            )
            return True
        except (ValueError, TypeError) as e:
            logger.error("Failed to write raw message for '%s': %s", topic, e)
            return False

    def close(self) -> None:
        """Finalize and close the MCAP file."""
        try:
            if self._writer is not None:
                self._writer.finish()
                logger.info("Wrote %d OSI messages to %s",
                            self._written_count, self._output_path)
        finally:
            self._writer = None
            self._osi_channel = None
            if self._file is not None:
                try:
                    self._file.close()
                except OSError:
                    pass
                self._file = None
            self._raw_channels.clear()
            _unpatch_zstd_level()

    def __enter__(self) -> MCAPExporter:
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def __del__(self) -> None:
        if self._writer is not None:
            import warnings
            warnings.warn(
                f"MCAPExporter for {self._output_path} was not closed — "
                "MCAP file may be corrupt (missing footer/index)",
                ResourceWarning,
                stacklevel=2,
            )

    @property
    def include_raw(self) -> bool:
        return self._include_raw

    @property
    def written_count(self) -> int:
        return self._written_count
