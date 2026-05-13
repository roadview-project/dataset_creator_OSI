from osi3.osi_sensordata_pb2 import SensorData


class Camera:
    """Converts ROS CompressedImage messages to OSI SensorData."""

    MESSAGE_TYPE = SensorData

    def __init__(self, config):
        self.number_of_pixels_vertical = config["image_resolution_vertical"]
        self.number_of_pixels_horizontal = config["image_resolution_horizontal"]
        self.position_x = config["mounting_position"]["x"]
        self.position_y = config["mounting_position"]["y"]
        self.position_z = config["mounting_position"]["z"]
        self.roll = config["mounting_position"]["roll"]
        self.pitch = config["mounting_position"]["pitch"]
        self.yaw = config["mounting_position"]["yaw"]

    def export(self, msg):
        """Build and return a SensorData protobuf from a ROS image message."""
        sensor_data = SensorData()
        sensor_view = sensor_data.sensor_view
        sensor_view.add()
        camera = sensor_view[0]

        camera.timestamp.seconds = msg.header.stamp.secs
        camera.timestamp.nanos = msg.header.stamp.nsecs

        camera.camera_sensor_view.add()
        camera.camera_sensor_view[0].image_data = msg.data
        camera.camera_sensor_view[0].view_configuration.ChannelFormat.Name = 6
        camera.camera_sensor_view[0].view_configuration.number_of_pixels_vertical = self.number_of_pixels_vertical
        camera.camera_sensor_view[0].view_configuration.number_of_pixels_horizontal = self.number_of_pixels_horizontal
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.x = self.position_x
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.y = self.position_y
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.z = self.position_z
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.roll = self.roll
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.pitch = self.pitch
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.yaw = self.yaw

        return sensor_data
