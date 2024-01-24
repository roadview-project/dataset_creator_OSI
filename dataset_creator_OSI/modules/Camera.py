import sys
import numpy as np
import math as m
import cv2
import time
import struct


class Camera():

    def __init__(self, f_path, config, PATH_TO_OSI):
        sys.path.insert(1, PATH_TO_OSI)
        from osi3.osi_sensordata_pb2 import SensorData
        self.SensorData = SensorData
        self.f_path = f_path
        self.number_of_pixels_vertical = config["image_resolution_vertical"]
        self.number_of_pixels_horizontal = config["image_resolution_horizontal"]
        self.position_x = config["mounting_position"]["x"]
        self.position_y = config["mounting_position"]["y"]
        self.position_z = config["mounting_position"]["z"]
        self.roll = config["mounting_position"]["roll"]
        self.pitch = config["mounting_position"]["pitch"]
        self.yaw = config["mounting_position"]["yaw"]

    def export(self, msg):
        sensor_data = self.SensorData()
        sensor_view = sensor_data.sensor_view
        sensor_view.add()
        camera = sensor_view[0]

        # setting TimeStamp to OSI
        camera.timestamp.seconds = msg.header.stamp.secs
        camera.timestamp.nanos = msg.header.stamp.nsecs

        # getting image from ROS message

        str_msg = msg.data
        # adding image data to osi
        camera.camera_sensor_view.add()
        camera.camera_sensor_view[0].image_data = str_msg
        # adding view_configuration data
        camera.camera_sensor_view[0].view_configuration.ChannelFormat.Name = 6
        camera.camera_sensor_view[0].view_configuration.number_of_pixels_vertical = self.number_of_pixels_vertical
        camera.camera_sensor_view[0].view_configuration.number_of_pixels_horizontal = self.number_of_pixels_horizontal
        # mouting position
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.x = self.position_x
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.y = self.position_y
        camera.camera_sensor_view[0].view_configuration.mounting_position.position.z = self.position_z
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.roll = self.roll
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.pitch = self.pitch
        camera.camera_sensor_view[0].view_configuration.mounting_position.orientation.yaw = self.yaw

        # naming and writing osi file
        osi_trace_file = "camera_sv_350_300.osi"
        with open(self.f_path + osi_trace_file, "ab+") as f:
            camera_ser = sensor_data.SerializeToString()
            f.write(struct.pack("<L", len(camera_ser)))
            f.write(camera_ser)
