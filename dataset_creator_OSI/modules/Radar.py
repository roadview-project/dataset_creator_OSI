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


class Radar():
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
        radar_data = sensor_data.feature_data.radar_sensor
        radar_data.add()
        # reading the data from the rosbag
        point_cloud_list = pc2.read_points_list(msg)

        # header of the osi radar
        radar_data[0].header.measurement_time.seconds = msg.header.stamp.secs
        radar_data[0].header.measurement_time.nanos = msg.header.stamp.nsecs
        radar_data[0].header.mounting_position.position.x = self.position_x
        radar_data[0].header.mounting_position.position.y = self.position_y
        radar_data[0].header.mounting_position.position.z = self.position_z
        radar_data[0].header.mounting_position.orientation.roll = self.roll
        radar_data[0].header.mounting_position.orientation.pitch = self.pitch
        radar_data[0].header.mounting_position.orientation.yaw = self.yaw
        # point cloud in te osi format
        for ind, point in enumerate(point_cloud_list):
            spherical = cart2sph(point.x, point.y, point.z)
            radar_data[0].detection.add()
            radar_data[0].detection[ind].position.distance = spherical[0]
            radar_data[0].detection[ind].position.elevation = spherical[1]
            radar_data[0].detection[ind].position.azimuth = spherical[2]
            radar_data[0].detection[ind].rcs = point.rcs
            radar_data[0].detection[ind].snr = point.snr
            radar_data[0].detection[ind].radial_velocity = point.velocity
        # Serialize
        osi_trace_file = "radar_sd_350_300"  # name of the file
        with open(self.f_path + osi_trace_file + ".osi", "ab+") as f:
            radar_ser = sensor_data.SerializeToString()  # serializing data
            # structuring the OSI data in order to put all the time stamps
            # in only one osi file
            f.write(struct.pack("<L", len(radar_ser)))
            f.write(radar_ser)
            f.close()
