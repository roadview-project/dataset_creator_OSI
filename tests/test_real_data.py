"""Tier 3: Real data tests — use actual sensor data from .playground/.

These tests are marked slow and skip if the data is not present.
Download the CEREMA Snow dataset to .playground/ to run them:

    mkdir -p .playground
    curl -C - -L -o .playground/ROADVIEW_SNOW.zip \
        "https://s3.ice.ri.se/roadview-WP3-Warwick/T3.2%20-%20Create%20Dataset/\
database/CE_dataset/ROADVIEW%20SNOW.zip"
    cd .playground && unzip ROADVIEW_SNOW.zip
"""

import struct
import sys
import types
from unittest.mock import patch, MagicMock

import pytest
from mcap.reader import make_reader
from osi3.osi_sensordata_pb2 import SensorData
from pathlib import Path
from rosbags.rosbag1 import Reader as BagReader
from rosbags.typesys import get_typestore, Stores

from dataset_creator_OSI.exporter import MCAPExporter

# Mock sensor_msgs before importing Radar (it does `import sensor_msgs.point_cloud2`)
if "sensor_msgs" not in sys.modules:
    _sm = MagicMock()
    sys.modules.setdefault("sensor_msgs", _sm)
    sys.modules.setdefault("sensor_msgs.point_cloud2", _sm.point_cloud2)

from dataset_creator_OSI.modules.Radar import Radar

PLAYGROUND_DIR = Path(__file__).resolve().parent.parent / ".playground"


def _find_ros_files():
    """Find all .bag and .ros files recursively in .playground/."""
    if not PLAYGROUND_DIR.exists():
        return []
    bags = list(PLAYGROUND_DIR.rglob("*.bag"))
    ros = list(PLAYGROUND_DIR.rglob("*.ros"))
    return sorted(bags + ros)


def _find_radar_bag():
    """Return the first .ros file with a PointCloud2 topic, or (None, None)."""
    for path in _find_ros_files():
        try:
            with BagReader(path) as reader:
                for topic, info in reader.topics.items():
                    if "PointCloud2" in info.msgtype:
                        return path, topic
        except Exception:
            continue
    return None, None


def _decode_pointcloud2(msg):
    """Decode PointCloud2 bytes into a list of SimpleNamespace points.

    Assumes float32 fields as discovered in the CEREMA Snow radar data:
    x(0), y(4), z(8), velocity(12), snr(16), power(20), rcs(24).
    """
    field_map = {f.name: f.offset for f in msg.fields}
    points = []
    for i in range(msg.width * msg.height):
        offset = i * msg.point_step
        chunk = msg.data[offset : offset + msg.point_step]
        point = types.SimpleNamespace(
            x=struct.unpack_from("<f", chunk, field_map.get("x", 0))[0],
            y=struct.unpack_from("<f", chunk, field_map.get("y", 4))[0],
            z=struct.unpack_from("<f", chunk, field_map.get("z", 8))[0],
            velocity=struct.unpack_from("<f", chunk, field_map.get("velocity", 12))[0],
            snr=struct.unpack_from("<f", chunk, field_map.get("snr", 16))[0],
            rcs=struct.unpack_from("<f", chunk, field_map.get("rcs", 24))[0],
            intensity=0.0,
            reflectivity=0.0,
        )
        points.append(point)
    return points


_ros_files = _find_ros_files()
_radar_bag, _radar_topic = _find_radar_bag()

requires_data = pytest.mark.skipif(
    not _ros_files,
    reason="No .bag/.ros files in .playground/ — download REHEARSE dataset",
)

requires_radar = pytest.mark.skipif(
    _radar_bag is None,
    reason="No radar PointCloud2 bag found in .playground/",
)


@requires_data
@pytest.mark.slow
class TestRealBagInspection:
    """Read a real bag and inspect its structure."""

    def test_bag_has_topics(self):
        with BagReader(_ros_files[0]) as reader:
            assert len(reader.topics) > 0

    def test_bag_has_pointcloud2_topic(self):
        with BagReader(_ros_files[0]) as reader:
            topic_types = {t: info.msgtype for t, info in reader.topics.items()}
            has_pc2 = any("PointCloud2" in v for v in topic_types.values())
            assert has_pc2, f"No PointCloud2 topics: {topic_types}"

    def test_bag_message_count_positive(self):
        with BagReader(_ros_files[0]) as reader:
            assert reader.message_count > 0


@requires_radar
@pytest.mark.slow
class TestRealRadarPipeline:
    """Feed real radar PointCloud2 messages through Radar.export and MCAPExporter."""

    def _make_radar_config(self):
        return {
            "sensor_type": "Radar",
            "topic": _radar_topic,
            "mounting_position": {
                "x": 3.5, "y": 0.0, "z": 0.5,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        }

    @patch("dataset_creator_OSI.modules.Radar.pc2")
    def test_export_first_message(self, mock_pc2):
        """Single real radar message round-trip: bag -> Radar.export -> SensorData."""
        ts = get_typestore(Stores.ROS1_NOETIC)
        radar = Radar(self._make_radar_config())

        with BagReader(_radar_bag) as reader:
            conns = [c for c in reader.connections if c.topic == _radar_topic]
            for conn, timestamp, rawdata in reader.messages(connections=conns):
                msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                points = _decode_pointcloud2(msg)
                mock_pc2.read_points_list.return_value = points

                adapted = types.SimpleNamespace(
                    header=types.SimpleNamespace(
                        stamp=types.SimpleNamespace(
                            secs=msg.header.stamp.sec,
                            nsecs=msg.header.stamp.nanosec,
                        ),
                        frame_id=msg.header.frame_id,
                    ),
                )

                sd = radar.export(adapted)
                assert isinstance(sd, SensorData)
                detections = sd.feature_data.radar_sensor[0].detection
                assert len(detections) == len(points)
                assert len(detections) > 0
                for det in detections:
                    assert det.position.distance >= 0
                break

    @patch("dataset_creator_OSI.modules.Radar.pc2")
    def test_export_batch_to_mcap(self, mock_pc2, mcap_path):
        """Export up to 10 real radar messages to MCAP and validate."""
        ts = get_typestore(Stores.ROS1_NOETIC)
        radar = Radar(self._make_radar_config())
        max_msgs = 10
        exported = 0

        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("radar", SensorData)
            with BagReader(_radar_bag) as reader:
                conns = [c for c in reader.connections if c.topic == _radar_topic]
                for conn, timestamp, rawdata in reader.messages(connections=conns):
                    if exported >= max_msgs:
                        break
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    points = _decode_pointcloud2(msg)
                    mock_pc2.read_points_list.return_value = points

                    adapted = types.SimpleNamespace(
                        header=types.SimpleNamespace(
                            stamp=types.SimpleNamespace(
                                secs=msg.header.stamp.sec,
                                nsecs=msg.header.stamp.nanosec,
                            ),
                            frame_id=msg.header.frame_id,
                        ),
                    )
                    sd = radar.export(adapted)
                    exporter.write_osi_message("radar", sd)
                    exported += 1

        assert exported == max_msgs

        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary.statistics.message_count == exported
            assert summary.statistics.schema_count >= 1

        data = mcap_path.read_bytes()
        assert data[:8] == b"\x89MCAP0\r\n"
        assert data[-8:] == b"\x89MCAP0\r\n"

    @patch("dataset_creator_OSI.modules.Radar.pc2")
    def test_exported_protobuf_deserializes(self, mock_pc2, mcap_path):
        """Every MCAP message deserializes to valid SensorData with detections."""
        ts = get_typestore(Stores.ROS1_NOETIC)
        radar = Radar(self._make_radar_config())

        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("radar", SensorData)
            with BagReader(_radar_bag) as reader:
                conns = [c for c in reader.connections if c.topic == _radar_topic]
                count = 0
                for conn, timestamp, rawdata in reader.messages(connections=conns):
                    if count >= 5:
                        break
                    msg = ts.deserialize_ros1(rawdata, conn.msgtype)
                    points = _decode_pointcloud2(msg)
                    mock_pc2.read_points_list.return_value = points

                    adapted = types.SimpleNamespace(
                        header=types.SimpleNamespace(
                            stamp=types.SimpleNamespace(
                                secs=msg.header.stamp.sec,
                                nsecs=msg.header.stamp.nanosec,
                            ),
                            frame_id=msg.header.frame_id,
                        ),
                    )
                    sd = radar.export(adapted)
                    exporter.write_osi_message("radar", sd)
                    count += 1

        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            for schema, channel, message in reader.iter_messages():
                sd = SensorData()
                sd.ParseFromString(message.data)
                assert sd.IsInitialized()
                assert len(sd.feature_data.radar_sensor) == 1
                assert len(sd.feature_data.radar_sensor[0].detection) > 0
