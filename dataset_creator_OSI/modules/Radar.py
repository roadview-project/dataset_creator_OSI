import sensor_msgs.point_cloud2 as pc2
from osi3.osi_sensordata_pb2 import SensorData

from dataset_creator_OSI.utils.coords import cart2sph


class Radar:
    """Converts ROS PointCloud2 messages to OSI SensorData (radar detections)."""

    MESSAGE_TYPE = SensorData

    def __init__(self, config):
        self.position_x = config["mounting_position"]["x"]
        self.position_y = config["mounting_position"]["y"]
        self.position_z = config["mounting_position"]["z"]
        self.roll = config["mounting_position"]["roll"]
        self.pitch = config["mounting_position"]["pitch"]
        self.yaw = config["mounting_position"]["yaw"]

    def export(self, msg):
        """Build and return a SensorData protobuf from a ROS PointCloud2 message."""
        sensor_data = SensorData()

        # Top-level fields for Lichtblick FrameTransforms
        sensor_data.timestamp.seconds = msg.header.stamp.secs
        sensor_data.timestamp.nanos = msg.header.stamp.nsecs
        sensor_data.mounting_position.position.x = self.position_x
        sensor_data.mounting_position.position.y = self.position_y
        sensor_data.mounting_position.position.z = self.position_z
        sensor_data.mounting_position.orientation.roll = self.roll
        sensor_data.mounting_position.orientation.pitch = self.pitch
        sensor_data.mounting_position.orientation.yaw = self.yaw

        radar_data = sensor_data.feature_data.radar_sensor
        radar_data.add()
        point_cloud_list = pc2.read_points_list(msg)

        radar_data[0].header.measurement_time.seconds = msg.header.stamp.secs
        radar_data[0].header.measurement_time.nanos = msg.header.stamp.nsecs
        radar_data[0].header.mounting_position.position.x = self.position_x
        radar_data[0].header.mounting_position.position.y = self.position_y
        radar_data[0].header.mounting_position.position.z = self.position_z
        radar_data[0].header.mounting_position.orientation.roll = self.roll
        radar_data[0].header.mounting_position.orientation.pitch = self.pitch
        radar_data[0].header.mounting_position.orientation.yaw = self.yaw

        for ind, point in enumerate(point_cloud_list):
            spherical = cart2sph(point.x, point.y, point.z)
            radar_data[0].detection.add()
            radar_data[0].detection[ind].position.distance = spherical[0]
            radar_data[0].detection[ind].position.elevation = spherical[1]
            radar_data[0].detection[ind].position.azimuth = spherical[2]
            radar_data[0].detection[ind].rcs = point.rcs
            radar_data[0].detection[ind].snr = point.snr
            radar_data[0].detection[ind].radial_velocity = point.velocity

        return sensor_data
