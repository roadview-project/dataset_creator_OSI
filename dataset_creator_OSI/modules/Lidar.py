import struct
import time
import sensor_msgs.point_cloud2 as pc2
import math as m
import rosbag
import sys


def cart2sph(x, y, z):
    XsqPlusYsq = x**2 + y**2
    r = m.sqrt(XsqPlusYsq + z**2)               # r
    elev = m.atan2(z, m.sqrt(XsqPlusYsq))     # theta
    az = m.atan2(y, x)                           # phi
    return r, elev, az


class Lidar():
    def __init__(self, f_path, config, PATH_TO_OSI):
        sys.path.insert(1, PATH_TO_OSI)
        from osi3.osi_sensordata_pb2 import SensorData
        self.SensorData = SensorData

        self.f_path = f_path
        self.position_x = config["mounting_position"]["x"]
        self.position_y = config["mounting_position"]["y"]
        self.position_z = config["mounting_position"]["z"]
        self.roll = config["mounting_position"]["roll"]
        self.pitch = config["mounting_position"]["pitch"]
        self.yaw = config["mounting_position"]["yaw"]

    def export(self, msg):
        ini = time.time()
        sensor_data = self.SensorData()
        lidar_data = sensor_data.feature_data.lidar_sensor
        lidar_data.add()
        point_cloud_list = pc2.read_points_list(
            msg)  # reading the data from the rosbag

        # header of the osi Lidar
        lidar_data[0].header.measurement_time.seconds = msg.header.stamp.secs
        lidar_data[0].header.measurement_time.nanos = msg.header.stamp.nsecs
        lidar_data[0].header.mounting_position.position.x = self.position_x
        lidar_data[0].header.mounting_position.position.y = self.position_y
        lidar_data[0].header.mounting_position.position.z = self.position_z
        lidar_data[0].header.mounting_position.orientation.roll = self.roll
        lidar_data[0].header.mounting_position.orientation.pitch = self.pitch
        lidar_data[0].header.mounting_position.orientation.yaw = self.yaw
        # point cloud in te osi format
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
            lidar_data[0].detection[ind].intensity = point.intensity
        # Serialize
        osi_trace_file = "lidar_sd_350_300"  # name of the file
        with open(self.f_path + osi_trace_file + ".osi", "ab+") as f:
            lidar_ser = sensor_data.SerializeToString()  # serializing data
            # structuring the OSI data in order to put all the time stamps
            # in only one osi file
            f.write(struct.pack("<L", len(lidar_ser)))
            f.write(lidar_ser)
            f.close()
