"""Tier 1: Unit tests — no ROS dependency required.

Tests cart2sph, build_static_target, MCAPExporter lifecycle, Camera.export,
and MCAP output validation using only protobuf and mcap (no rosbag/sensor_msgs).
"""

import math
import types
import warnings

import pytest
from mcap.reader import make_reader
from osi3.osi_sensordata_pb2 import SensorData
from osi3.osi_groundtruth_pb2 import GroundTruth

from dataset_creator_OSI.utils.coords import cart2sph
from dataset_creator_OSI.modules.static_target import build_static_target
from dataset_creator_OSI.exporter import MCAPExporter


# ---------------------------------------------------------------------------
# cart2sph
# ---------------------------------------------------------------------------

class TestCart2Sph:
    def test_origin(self):
        r, elev, az = cart2sph(0, 0, 0)
        assert r == 0.0

    def test_unit_x(self):
        r, elev, az = cart2sph(1, 0, 0)
        assert r == pytest.approx(1.0)
        assert elev == pytest.approx(0.0)
        assert az == pytest.approx(0.0)

    def test_unit_y(self):
        r, elev, az = cart2sph(0, 1, 0)
        assert r == pytest.approx(1.0)
        assert elev == pytest.approx(0.0)
        assert az == pytest.approx(math.pi / 2)

    def test_unit_z(self):
        r, elev, az = cart2sph(0, 0, 1)
        assert r == pytest.approx(1.0)
        assert elev == pytest.approx(math.pi / 2)

    def test_known_diagonal(self):
        r, elev, az = cart2sph(1, 1, 1)
        assert r == pytest.approx(math.sqrt(3))
        assert elev == pytest.approx(math.atan2(1, math.sqrt(2)))
        assert az == pytest.approx(math.pi / 4)

    def test_negative_coords(self):
        r, elev, az = cart2sph(-1, -1, 0)
        assert r == pytest.approx(math.sqrt(2))
        assert az == pytest.approx(-3 * math.pi / 4)


# ---------------------------------------------------------------------------
# build_static_target
# ---------------------------------------------------------------------------

class TestBuildStaticTarget:
    def test_dimensions_and_position(self):
        gt = build_static_target(1.0, 2.0, 1.5, 10.0, 20.0, 0, 0, 0.5,
                                 timestamp_s=1000.0)
        obj = gt.stationary_object[0]
        assert obj.base.dimension.width == 1.0
        assert obj.base.dimension.length == 2.0
        assert obj.base.dimension.height == 1.5
        assert obj.base.position.x == 10.0
        assert obj.base.position.y == 20.0
        # z = height/2 for bounding-box center of ground-resting object
        assert obj.base.position.z == pytest.approx(0.75)
        assert obj.base.orientation.yaw == 0.5

    def test_integer_timestamp(self):
        gt = build_static_target(1, 1, 1, 0, 0, 0, 0, 0, timestamp_s=1000.0)
        assert gt.timestamp.seconds == 1000
        assert gt.timestamp.nanos == 0

    def test_fractional_timestamp(self):
        gt = build_static_target(1, 1, 1, 0, 0, 0, 0, 0, timestamp_s=1000.5)
        assert gt.timestamp.seconds == 1000
        assert gt.timestamp.nanos == pytest.approx(500_000_000, abs=1000)

    def test_has_exactly_one_object(self):
        gt = build_static_target(1, 1, 1, 0, 0, 0, 0, 0, timestamp_s=0.0)
        assert len(gt.stationary_object) == 1

    def test_host_vehicle_id_default(self):
        gt = build_static_target(1, 1, 1, 0, 0, 0, 0, 0, timestamp_s=0.0)
        assert gt.host_vehicle_id.value == 0
        # Must reference an actual moving_object entry
        assert len(gt.moving_object) == 1
        assert gt.moving_object[0].id.value == 0

    def test_host_vehicle_id_custom(self):
        gt = build_static_target(1, 1, 1, 0, 0, 0, 0, 0, timestamp_s=0.0,
                                 host_vehicle_id=42)
        assert gt.host_vehicle_id.value == 42
        assert gt.moving_object[0].id.value == 42


# ---------------------------------------------------------------------------
# MCAPExporter
# ---------------------------------------------------------------------------

class TestMCAPExporter:
    def test_context_manager_creates_valid_file(self, mcap_path):
        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("sensor", SensorData)
            sd = SensorData()
            sd.sensor_view.add()
            sd.sensor_view[0].timestamp.seconds = 100
            sd.sensor_view[0].timestamp.nanos = 500_000_000
            exporter.write_osi_message("sensor", sd)

        assert mcap_path.exists()
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary is not None
            assert summary.statistics.message_count >= 1

    def test_double_open_raises(self, mcap_path):
        exporter = MCAPExporter(mcap_path)
        exporter.open()
        try:
            with pytest.raises(RuntimeError, match="already open"):
                exporter.open()
        finally:
            exporter.close()

    def test_write_when_not_open_returns_false_and_warns(self, mcap_path, caplog):
        exporter = MCAPExporter(mcap_path)
        ok = exporter.write_osi_message("test", SensorData())
        assert not ok
        assert "not open" in caplog.text

    def test_written_count(self, mcap_path):
        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("s", SensorData)
            for i in range(5):
                sd = SensorData()
                sd.sensor_view.add()
                sd.sensor_view[0].timestamp.seconds = i
                exporter.write_osi_message("s", sd)
            assert exporter.written_count == 5

    def test_resource_warning_on_unclosed(self, mcap_path):
        exporter = MCAPExporter(mcap_path)
        exporter.open()
        exporter.add_osi_channel("s", SensorData)
        with pytest.warns(ResourceWarning, match="was not closed"):
            exporter.__del__()
        # Clean up to avoid actual file corruption
        exporter._writer = None  # prevent double warning
        exporter.close()

    def test_groundtruth_nonzero_timestamp(self, mcap_path):
        """Verify GroundTruth with timestamp produces non-zero MCAP log_time."""
        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("gt", GroundTruth)
            gt = build_static_target(1, 2, 1.5, 10, 20, 0, 0, 0.5,
                                     timestamp_s=1000.5)
            exporter.write_osi_message("gt", gt)

        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary.statistics.message_start_time > 0

    def test_include_raw_property(self, mcap_path):
        exporter = MCAPExporter(mcap_path, include_raw=True)
        assert exporter.include_raw is True
        exporter2 = MCAPExporter(mcap_path, include_raw=False)
        assert exporter2.include_raw is False


# ---------------------------------------------------------------------------
# Camera.export (with mock ROS message)
# ---------------------------------------------------------------------------

class TestCameraExport:
    def _make_camera(self):
        from dataset_creator_OSI.modules.Camera import Camera
        config = {
            "image_resolution_vertical": 480,
            "image_resolution_horizontal": 640,
            "mounting_position": {
                "x": 1.0, "y": 0.0, "z": 1.5,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        }
        return Camera(config)

    def _make_mock_msg(self, secs=100, nsecs=500):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(secs=secs, nsecs=nsecs),
            ),
            data=b"\xff\xd8fake_jpeg_data",
        )

    def test_export_returns_sensor_data(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg())
        assert isinstance(result, SensorData)

    def test_timestamp_propagated(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg(secs=42, nsecs=123))
        assert result.sensor_view[0].timestamp.seconds == 42
        assert result.sensor_view[0].timestamp.nanos == 123
        # Top-level timestamp for MCAP log_time
        assert result.timestamp.seconds == 42
        assert result.timestamp.nanos == 123

    def test_image_data_preserved(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg())
        assert result.sensor_view[0].camera_sensor_view[0].image_data == b"\xff\xd8fake_jpeg_data"

    def test_channel_format_is_other(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg())
        fmt = result.sensor_view[0].camera_sensor_view[0].view_configuration.channel_format
        assert len(fmt) == 1
        assert fmt[0] == 1  # CHANNEL_FORMAT_OTHER (compressed ROS image)

    def test_resolution_set(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg())
        vc = result.sensor_view[0].camera_sensor_view[0].view_configuration
        assert vc.number_of_pixels_vertical == 480
        assert vc.number_of_pixels_horizontal == 640

    def test_mounting_position(self):
        camera = self._make_camera()
        result = camera.export(self._make_mock_msg())
        # Nested view config mounting position
        mp = result.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position
        assert mp.position.x == 1.0
        assert mp.position.z == 1.5
        # Top-level SensorData mounting position (Lichtblick FrameTransforms)
        assert result.mounting_position.position.x == 1.0
        assert result.mounting_position.position.z == 1.5
