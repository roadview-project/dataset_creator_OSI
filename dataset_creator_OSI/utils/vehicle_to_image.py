import numpy as np
from math import radians, cos, sin


class Camera(moving_obj):
    """this Camera class was developed by making a traduction from the matlab code, important aspects are:
        pitch roll yaw are in degrees,
        sensor location is x,y location in m
        height is z height in m
        intrinsics is matlab intrisics, with is diferent then the one calculated in opencv by on transpose"""

    def __init__(self, intrinsics, pitch, roll, yaw, height, sensor_location):
        super(Camera, self).__init__()
        self.intrinsics = intrinsics
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.height = height
        self.sensor_location = sensor_location

    def vehicle_to_image(self, vehicle_points):
        vehicle_points = np.pad(vehicle_points, ((0, 0), (0, 1)), constant_values=(1))
        image_points = vehicle_points @ self.rawTformToImage3D()
        image_points[:, 0] = image_points[:, 0] / image_points[:, 2]
        image_points[:, 1] = image_points[:, 1] / image_points[:, 2]
        return image_points[:, :2]

    def rawTformToImage3D(self):
        translation = self.translation_vector()
        rotation = self.rotation_matrix()
        tform = np.vstack((rotation, translation)) @ self.intrinsics
        return tform

    def translation_vector(self):
        rotation_matrix = self.rotZ(-self.yaw) @ self.rotX(90 - self.pitch) @ self.rotZ(self.roll)
        sl = self.sensor_location

        translation_in_world_units = np.array([sl[1], sl[0], self.height])
        translation = translation_in_world_units @ rotation_matrix
        return translation

    def rotation_matrix(self):
        rotation = self.rotY(180) @ self.rotZ(-90) @ self.rotZ(-self.yaw) @ self.rotX(90 - self.pitch) @ self.rotZ(self.roll)
        return rotation

    def rotX(self, a):
        a = radians(a)
        R = np.array([[1, 0, 0],
                      [0, cos(a), -sin(a)],
                      [0, sin(a), cos(a)]])
        return R

    def rotY(self, a):
        a = radians(a)
        R = np.array([[cos(a), 0, sin(a)],
                      [0, 1, 0],
                      [-sin(a), 0, cos(a)]])
        return R

    def rotZ(self, a):
        a = radians(a)
        R = np.array([[cos(a), -sin(a), 0],
                      [sin(a), cos(a), 0],
                      [0, 0, 1]])
        return R
