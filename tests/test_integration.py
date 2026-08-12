"""Tier 2: Integration tests — mock ROS dependencies, test full pipeline flow.

Tests instantiate_classes, read_bag, and MCAPExporter working together
end-to-end with mock Bag and sensor_msgs objects.
"""

import sys
import types
import logging
from unittest.mock import patch, MagicMock
from pathlib import Path

import pytest
import yaml
from mcap.reader import make_reader
from osi3.osi_sensordata_pb2 import SensorData

# ---------------------------------------------------------------------------
# Mock ROS modules before importing anything that depends on them
# ---------------------------------------------------------------------------
_rosbag_mock = MagicMock()
_sensor_msgs_mock = MagicMock()

# Only inject mocks if the real modules aren't available
if "rosbag" not in sys.modules:
    sys.modules.setdefault("rosbag", _rosbag_mock)
    sys.modules.setdefault("rosbag.bag", _rosbag_mock.bag)
if "sensor_msgs" not in sys.modules:
    sys.modules.setdefault("sensor_msgs", _sensor_msgs_mock)
    sys.modules.setdefault("sensor_msgs.point_cloud2", _sensor_msgs_mock.point_cloud2)

from dataset_creator_OSI.exporter import MCAPExporter
from dataset_creator import instantiate_classes, read_bag


# ---------------------------------------------------------------------------
# Mock helpers
# ---------------------------------------------------------------------------

def _make_stamp(secs, nsecs):
    return types.SimpleNamespace(secs=secs, nsecs=nsecs)


def _make_header(secs, nsecs, frame_id="test"):
    return types.SimpleNamespace(
        stamp=_make_stamp(secs, nsecs),
        frame_id=frame_id,
    )


def _make_camera_msg(secs=100, nsecs=0):
    return types.SimpleNamespace(
        header=_make_header(secs, nsecs, "camera"),
        data=b"\xff\xd8fake_jpeg",
    )


def _make_point(x, y, z, intensity=1.0, reflectivity=0.5,
                rcs=0.1, snr=10.0, velocity=0.0):
    return types.SimpleNamespace(
        x=x, y=y, z=z,
        intensity=intensity,
        reflectivity=reflectivity,
        rcs=rcs,
        snr=snr,
        velocity=velocity,
    )


def _make_pc2_msg(secs=100, nsecs=0, points=None):
    """Mock a PointCloud2 message."""
    if points is None:
        points = [_make_point(1.0, 2.0, 3.0)]
    msg = types.SimpleNamespace(
        header=_make_header(secs, nsecs, "lidar"),
    )
    msg._points = points  # stash for the mock read_points_list
    return msg


class MockBag:
    """Minimal mock for rosbag.Bag."""

    def __init__(self, messages, name="test.bag"):
        self._messages = messages  # list of (topic, msg, time)
        self._file = types.SimpleNamespace(name=name)

    def read_messages(self, topics=None):
        if topics is not None:
            return iter([(t, m, ts) for t, m, ts in self._messages if t in topics])
        return iter(self._messages)

    def get_message_count(self, topic_filters=None):
        if topic_filters is not None:
            return sum(1 for t, _, _ in self._messages if t in topic_filters)
        return len(self._messages)

    def get_start_time(self):
        if self._messages:
            return self._messages[0][2].to_sec()
        return 0.0

    def get_type_and_topic_info(self):
        topics = {}
        for topic, msg, t in self._messages:
            if topic not in topics:
                topics[topic] = types.SimpleNamespace(msg_type="mock/Message")
        return (None, topics)


class MockTime:
    """Minimal mock for rospy.Time."""

    def __init__(self, secs, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs

    def to_nsec(self):
        return self.secs * 1_000_000_000 + self.nsecs

    def to_sec(self):
        return self.secs + self.nsecs / 1e9


# ---------------------------------------------------------------------------
# Config fixture
# ---------------------------------------------------------------------------

@pytest.fixture
def sample_config(tmp_path):
    config = {
        "front_camera": {
            "sensor_type": "Camera",
            "topic": "/camera/compressed",
            "image_resolution_vertical": 480,
            "image_resolution_horizontal": 640,
            "mounting_position": {
                "x": 1.5, "y": 0.0, "z": 1.2,
                "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            },
        },
    }
    config_path = tmp_path / "config.yaml"
    config_path.write_text(yaml.dump(config))
    return config, config_path


@pytest.fixture
def sample_targets(tmp_path):
    targets = {
        "targets": {
            "pedestrian": {"width": 0.5, "length": 0.3, "height": 1.8},
            "car": {"width": 1.8, "length": 4.5, "height": 1.5},
        }
    }
    targets_path = tmp_path / "targets.yaml"
    targets_path.write_text(yaml.dump(targets))
    return targets_path


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestInstantiateClasses:
    def test_loads_camera(self, sample_config):
        config, _ = sample_config
        classes = instantiate_classes(config)
        assert "/camera/compressed" in classes
        assert classes["/camera/compressed"].MESSAGE_TYPE == SensorData

    def test_message_type_attribute(self, sample_config):
        config, _ = sample_config
        classes = instantiate_classes(config)
        sensor = classes["/camera/compressed"]
        assert hasattr(sensor, "MESSAGE_TYPE")
        assert hasattr(sensor, "export")


class TestReadBagIntegration:
    """Test read_bag with a mock bag, real sensor classes, and real MCAPExporter."""

    def test_camera_pipeline(self, sample_config, mcap_path):
        config, _ = sample_config
        sensor_classes = instantiate_classes(config)

        messages = [
            ("/camera/compressed", _make_camera_msg(100, 0), MockTime(100)),
            ("/camera/compressed", _make_camera_msg(101, 0), MockTime(101)),
            ("/camera/compressed", _make_camera_msg(102, 0), MockTime(102)),
        ]
        bag = MockBag(messages)
        logger = logging.getLogger("test_integration")

        import osi3
        osi_version = str(osi3.__version__)
        channel_meta = {"net.asam.osi.trace.channel.osi_version": osi_version}

        with MCAPExporter(mcap_path) as exporter:
            for sensor_name in config:
                topic = config[sensor_name]["topic"]
                sensor = sensor_classes[topic]
                exporter.add_osi_channel(sensor_name, sensor.MESSAGE_TYPE,
                                         metadata=channel_meta)
            read_bag(bag, config, sensor_classes, exporter, logger)

        # Validate output
        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary.statistics.message_count == 3
            # NOTE: Camera.export() doesn't set top-level SensorData.timestamp
            # (pre-existing issue), so MCAP log_time=0. Verify messages are
            # written; timestamp correctness is a follow-up fix.

    @patch("dataset_creator_OSI.modules.Lidar.pc2")
    def test_lidar_pipeline(self, mock_pc2, mcap_path):
        """Test lidar with mocked sensor_msgs.point_cloud2."""
        config = {
            "top_lidar": {
                "sensor_type": "Lidar",
                "topic": "/lidar/points",
                "mounting_position": {
                    "x": 0.0, "y": 0.0, "z": 2.0,
                    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                },
            },
        }

        points = [_make_point(1.0, 2.0, 3.0), _make_point(4.0, 5.0, 6.0)]
        mock_pc2.read_points_list.return_value = points

        sensor_classes = instantiate_classes(config)

        messages = [
            ("/lidar/points", _make_pc2_msg(100, 0, points), MockTime(100)),
            ("/lidar/points", _make_pc2_msg(101, 0, points), MockTime(101)),
        ]
        bag = MockBag(messages)
        logger = logging.getLogger("test_integration")

        import osi3
        osi_version = str(osi3.__version__)
        channel_meta = {"net.asam.osi.trace.channel.osi_version": osi_version}

        with MCAPExporter(mcap_path) as exporter:
            for sensor_name in config:
                topic = config[sensor_name]["topic"]
                sensor = sensor_classes[topic]
                exporter.add_osi_channel(sensor_name, sensor.MESSAGE_TYPE,
                                         metadata=channel_meta)
            read_bag(bag, config, sensor_classes, exporter, logger)

        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            assert summary.statistics.message_count == 2


class TestMCAPOutputCompliance:
    """Validate MCAP file structure and OSI metadata."""

    def test_has_osi_trace_metadata(self, mcap_path):
        with MCAPExporter(mcap_path, description="test file") as exporter:
            exporter.add_osi_channel("s", SensorData)
            sd = SensorData()
            sd.sensor_view.add()
            sd.sensor_view[0].timestamp.seconds = 1
            exporter.write_osi_message("s", sd)

        with open(mcap_path, "rb") as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            osi_meta = None
            for m in reader.iter_metadata():
                if m.name == "net.asam.osi.trace":
                    osi_meta = m
                    break
            assert osi_meta is not None
            assert "description" in osi_meta.metadata
            assert osi_meta.metadata["description"] == "test file"

    def test_file_has_magic_bytes(self, mcap_path):
        """MCAP files must start and end with magic bytes."""
        with MCAPExporter(mcap_path) as exporter:
            exporter.add_osi_channel("s", SensorData)
            sd = SensorData()
            sd.sensor_view.add()
            sd.sensor_view[0].timestamp.seconds = 1
            exporter.write_osi_message("s", sd)

        data = mcap_path.read_bytes()
        MCAP_MAGIC = b"\x89MCAP0\r\n"
        assert data[:len(MCAP_MAGIC)] == MCAP_MAGIC
        assert data[-len(MCAP_MAGIC):] == MCAP_MAGIC
