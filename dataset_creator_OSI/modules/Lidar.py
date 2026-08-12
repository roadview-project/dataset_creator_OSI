import sensor_msgs.point_cloud2 as pc2
from osi3.osi_sensordata_pb2 import SensorData

from dataset_creator_OSI.utils.coords import cart2sph


class Lidar:
    """Converts ROS PointCloud2 messages to OSI SensorData (lidar detections)."""

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

        lidar_data = sensor_data.feature_data.lidar_sensor
        lidar_data.add()
        point_cloud_list = pc2.read_points_list(msg)

        lidar_data[0].header.measurement_time.seconds = msg.header.stamp.secs
        lidar_data[0].header.measurement_time.nanos = msg.header.stamp.nsecs
        lidar_data[0].header.mounting_position.position.x = self.position_x
        lidar_data[0].header.mounting_position.position.y = self.position_y
        lidar_data[0].header.mounting_position.position.z = self.position_z
        lidar_data[0].header.mounting_position.orientation.roll = self.roll
        lidar_data[0].header.mounting_position.orientation.pitch = self.pitch
        lidar_data[0].header.mounting_position.orientation.yaw = self.yaw

        for ind, point in enumerate(point_cloud_list):
            spherical = cart2sph(point.x, point.y, point.z)
            lidar_data[0].detection.add()
            lidar_data[0].detection[ind].position.distance = spherical[0]
            lidar_data[0].detection[ind].position.elevation = spherical[1]
            lidar_data[0].detection[ind].position.azimuth = spherical[2]
            try:
                lidar_data[0].detection[ind].reflectivity = point.reflectivity
            except AttributeError:
                lidar_data[0].detection[ind].reflectivity = 0
            try:
                lidar_data[0].detection[ind].intensity = point.intensity
            except AttributeError:
                lidar_data[0].detection[ind].intensity = 0

        return sensor_data
