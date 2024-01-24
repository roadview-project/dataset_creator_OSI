from osi3.osi_datarecording_pb2 import SensorDataSeries
from osi3.osi_sensordata_pb2 import SensorData
with open('TWICE_path.txt', 'r') as f:
    path_twice = f.read()
import math as m
import numpy as np
import cv2
from vehicle_to_image import Camera
import json
import ast
import google.protobuf.message
import google.protobuf.json_format
import sys
import struct
import os
import pathlib
sys.path.insert(1, os.path.join(path_twice, "TWICE", "open-simulation-interface"))


def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1 * np.pi / 180)
    s1 = np.sin(theta1 * np.pi / 180)
    c2 = np.cos(theta2 * np.pi / 180)
    s2 = np.sin(theta2 * np.pi / 180)
    c3 = np.cos(theta3 * np.pi / 180)
    s3 = np.sin(theta3 * np.pi / 180)
    if order == 'xyz':
        matrix = np.array([[c2 * c3, -c2 * s3, s2],
                           [c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3, -c2 * s1],
                           [s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3, c1 * c2]])
    return matrix


def cuboid_corners(cuboid):
    """
 0--------3
/ |     /  |
1------2   |
| 4 ---|-- 7       Indexes of cPoints and ImagePoints matrix
| /     | /                   ^
5------6         /    <--     |                 
                x      y     z

                phi is the angle between y and x
                theta is the angle between x and z
                x is pointed as the fowards
"""

    x, y, z, roll, pitch, yaw, length, width, height = cuboid[0], cuboid[1], cuboid[2], cuboid[3], cuboid[4], cuboid[5], cuboid[6], cuboid[7], cuboid[8]
    dimensions = [length, width, height]
    cPoints = np.array([[dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2],
                        [-dimensions[0] / 2, dimensions[1] / 2, dimensions[2] / 2],
                        [-dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2],
                        [dimensions[0] / 2, -dimensions[1] / 2, dimensions[2] / 2],
                        [dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2],
                        [-dimensions[0] / 2, dimensions[1] / 2, -dimensions[2] / 2],
                        [-dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2],
                        [dimensions[0] / 2, -dimensions[1] / 2, -dimensions[2] / 2]])
    rot_matrix = rotation_matrix(roll, pitch, yaw)
    for j in range(cPoints.shape[0]):
        # making the rotation
        cPoints[j] = np.matmul(rot_matrix, cPoints[j])
        # adjusting the position
        cPoints[j][0] = cPoints[j][0] + x
        cPoints[j][1] = cPoints[j][1] + y
        cPoints[j][2] = cPoints[j][2] + z
    return cPoints


def read_adma_ego(path_ego_osi):
    obj_ego = SensorDataSeries()
    with open(path_ego_osi, 'rb') as f:
        obj_ego.ParseFromString(f.read())
    return obj_ego


def open_label_read(path, frame, list_images, obj_ego, cbla, synthetic, OTA, without):
    f = open(path)
    data = json.load(f)
    # getting cuboid from json
    cuboid_1 = data['openlabel']['frames'][frame]['objects'][0]['object_data']['cuboid']['val']
    # getting camera matrix from json
    camera_matrix = data['openlabel']['frames'][frame]['frame_properties']['Streams']['Camera1']['stream_properties']['intrinsics_pinhole']['camera_matrix_3x4']
    # geting IMU ego index
    ego_index = data['openlabel']['frames'][frame]['frame_properties']['Streams']['IMU_ego']['stream_properties']['sync']['frame_stream'] - 1
    # getting ego angles to compensate the motion
    roll_ego = obj_ego.sensor_data[ego_index].sensor_view[0].host_vehicle_data.vehicle_motion.orientation.roll
    pitch_ego = obj_ego.sensor_data[ego_index].sensor_view[0].host_vehicle_data.vehicle_motion.orientation.pitch
    yaw_ego = obj_ego.sensor_data[ego_index].sensor_view[0].host_vehicle_data.vehicle_motion.orientation.yaw
    # getting cuboid corners
    points_1 = cuboid_corners(cuboid_1)

    # projecting 3D point into 2D image
    height = list_images[frame].z
    #height = 1.27845668792725;
    roll = list_images[frame].roll + m.degrees(roll_ego)
    #roll = 0.466911673545837
    pitch = list_images[frame].pitch + m.degrees(pitch_ego)

    #pitch = 3.3
    yaw = list_images[frame].yaw + 0.1

    if synthetic:
        yaw = 0.95
        pitch = 3.69 + m.degrees(pitch_ego)
        height = 1.1992
    if OTA:
        yaw = 2.82
        pitch = 7.27 + m.degrees(pitch_ego)
        height = 1.1992
    # if without and OTA:
        # yaw=5.6
    sensor_location = np.array([list_images[frame].x, list_images[frame].y])
    #sensor_location  = np.array([1.31996130943298, -0.071691632270813])
    intrinsic_matrix = np.array([[camera_matrix[0][0], 0, 0], [0, camera_matrix[1][1], 0], [camera_matrix[0][2], camera_matrix[1][2], 1]])
    # Creating moving_obj of vehicle_to_image with projection parameters
    camera = Camera(intrinsic_matrix, pitch, roll, yaw, height, sensor_location)
    # projecting points
    xyImagePoints = camera.vehicle_to_image(points_1)
    # CBLA need to plot two cuboids
    if cbla:
        cuboid_2 = data['openlabel']['frames'][frame]['objects'][1]['object_data']['cuboid']['val']
        points_2 = cuboid_corners(cuboid_2)
        xyImagePoints_2 = camera.vehicle_to_image(points_2)
        return(xyImagePoints, xyImagePoints_2)
    else:
        return(xyImagePoints)


class Camera_project():
    def __init__(self, img, timestamp, x, y, z, roll, pitch, yaw):
        self.img = img
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def instantiate_camera(img, timestamp, x, y, z, roll, pitch, yaw):
        roll_deg = m.degrees(roll)
        pitch_deg = m.degrees(pitch)
        yaw_deg = m.degrees(yaw)
        obj = Camera_project(img, timestamp, x, y, z, roll_deg, pitch_deg, yaw_deg)
        return(obj)

    def open_osi_image(path_osi_file):    
        cam_osi = SensorData()
        list_images = []
        with open(path_osi_file, 'rb') as f:
            while 1:
                message_size_bytes = f.read(struct.calcsize("<L"))
                if len(message_size_bytes) == 0:
                    break
                message_size = struct.unpack("<L", message_size_bytes)[0]
                message_bytes = f.read(message_size)
                cam_osi.ParseFromString(message_bytes)
                # image data
                str_msg = cam_osi.sensor_view[0].camera_sensor_view[0].image_data
                cv_img = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=str_msg)
                im = cv2.imdecode(cv_img, cv2.IMREAD_ANYCOLOR)
                # image and camera attributes
                timestamp = (cam_osi.sensor_view[0].timestamp.seconds) + (cam_osi.sensor_view[0].timestamp.nanos / 1000000000)
                x = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.position.x
                y = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.position.y
                z = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.position.z
                roll = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.orientation.roll
                pitch = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.orientation.pitch
                yaw = cam_osi.sensor_view[0].camera_sensor_view[0].view_configuration.mounting_position.orientation.yaw
                # putting data in a list
                obj = Camera_project.instantiate_camera(im, timestamp, x, y, z, roll, pitch, yaw)
                list_images.append(obj)
            return (list_images)

    def project_cuboid_image(path, frame, projectCuboid, showImage, showVideo, saveImage, showFrame, showTimestamp, pathSaveImage=""):
        # reading camera osi file
        list_images = Camera_project.open_osi_image(path)

        # reading json Openlabel file
        path_openlabel = os.path.join(os.path.dirname(path), "open_label_camera.json")

        # reading .osi ego file
        path_ego = os.path.join(os.path.dirname(os.path.dirname(path)), "IMU_ego", "ego_sv_350_300.osi")
        obj_ego = read_adma_ego(path_ego)

        # name of the video and image file
        path_name = pathlib.PurePath(path)
        list_elements = list(path_name.parts)
        name_file = list_elements[-7] + "_" + list_elements[-6] + "_" + list_elements[-5] + "_" + list_elements[-4] + "_" + list_elements[-3]
        name_file = str(name_file)

        synthetic = False
        cbla = False
        OTA = False
        without = False
        if ("CBLA" and "with_car") in list_elements:
            cbla = True
            name_file = list_elements[-8] + list_elements[-7] + "_" + list_elements[-6] + "_" + list_elements[-5] + "_" + list_elements[-4] + "_" + list_elements[-3]
        if "synthetic" in list_elements:
            synthetic = True
            name_file = list_elements[-7] + "_" + list_elements[-6] + "_" + list_elements[-5] + "_" + list_elements[-4] + "_" + list_elements[-3] + "_" + list_elements[-2]
            name_file = str(name_file)
        if synthetic and cbla:
            name_file = list_elements[-8] + list_elements[-7] + "_" + list_elements[-6] + "_" + list_elements[-5] + "_" + list_elements[-4] + "_" + list_elements[-3] + "_" + list_elements[-2]
        if "Camera_OTA" in list_elements:
            OTA = True
        if "without_car" in list_elements:
            without = True

        img = list_images[frame].img

        # reading image data
        if projectCuboid: 
            for i in range(len(list_images)):
                # read image
                img = list_images[i].img 
                overlay = img.copy()   

                # read openlabel cuboid and ego position. Return the projected points
                if cbla:
                    xyImagePoints, xyImagePoints_2 = open_label_read(path_openlabel, int(i), list_images, obj_ego, cbla, synthetic, OTA, without)
                else:
                    xyImagePoints = open_label_read(path_openlabel, int(i), list_images, obj_ego, cbla, synthetic, OTA, without)
                # projecting 3D point into 2D image
                rect_front = np.array([xyImagePoints[1], xyImagePoints[2], xyImagePoints[6], xyImagePoints[5]], np.int32)
                rect_front = rect_front.reshape((-1, 1, 2))
                rect_back = np.array([xyImagePoints[0], xyImagePoints[3], xyImagePoints[7], xyImagePoints[4]], np.int32)
                rect_back = rect_back.reshape((-1, 1, 2))
                img = cv2.polylines(img, [rect_back], True, (255, 150, 0), 2, -1)
                img = cv2.line(img, (int(xyImagePoints[1][0]), int(xyImagePoints[1][1])), (int(xyImagePoints[0][0]), int(xyImagePoints[0][1])), (255, 150, 0), 2)
                img = cv2.line(img, (int(xyImagePoints[3][0]), int(xyImagePoints[3][1])), (int(xyImagePoints[2][0]), int(xyImagePoints[2][1])), (255, 150, 0), 2)
                img = cv2.line(img, (int(xyImagePoints[5][0]), int(xyImagePoints[5][1])), (int(xyImagePoints[4][0]), int(xyImagePoints[4][1])), (255, 150, 0), 2)
                img = cv2.line(img, (int(xyImagePoints[7][0]), int(xyImagePoints[7][1])), (int(xyImagePoints[6][0]), int(xyImagePoints[6][1])), (255, 150, 0), 2)
                img = cv2.polylines(img, [rect_front], True, (255, 0, 0), 2, -1)
                cv2.fillPoly(overlay, pts=[rect_front], color=(255, 100, 0))
                alpha = 0.2  # Transparency factor
                img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
                img = cv2.circle(img, (int(xyImagePoints[0][0]), int(xyImagePoints[0][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[1][0]), int(xyImagePoints[1][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[2][0]), int(xyImagePoints[2][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[3][0]), int(xyImagePoints[3][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[4][0]), int(xyImagePoints[4][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[5][0]), int(xyImagePoints[5][1])), radius=5, color=(255, 255, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[6][0]), int(xyImagePoints[6][1])), radius=5, color=(255, 0, 0), thickness=-1)
                img = cv2.circle(img, (int(xyImagePoints[7][0]), int(xyImagePoints[7][1])), radius=5, color=(255, 0, 0), thickness=-1)

                if cbla:
                    # projecting 3D point into 2D image
                    rect_front = np.array([xyImagePoints_2[1], xyImagePoints_2[2], xyImagePoints_2[6], xyImagePoints_2[5]], np.int32)
                    rect_front = rect_front.reshape((-1, 1, 2))
                    rect_back = np.array([xyImagePoints_2[0], xyImagePoints_2[3], xyImagePoints_2[7], xyImagePoints_2[4]], np.int32)
                    rect_back = rect_back.reshape((-1, 1, 2))
                    img = cv2.polylines(img, [rect_back], True, (255, 150, 0), 2, -1)
                    img = cv2.line(img, (int(xyImagePoints_2[1][0]), int(xyImagePoints_2[1][1])), (int(xyImagePoints_2[0][0]), int(xyImagePoints_2[0][1])), (255, 150, 0), 2)
                    img = cv2.line(img, (int(xyImagePoints_2[3][0]), int(xyImagePoints_2[3][1])), (int(xyImagePoints_2[2][0]), int(xyImagePoints_2[2][1])), (255, 150, 0), 2)
                    img = cv2.line(img, (int(xyImagePoints_2[5][0]), int(xyImagePoints_2[5][1])), (int(xyImagePoints_2[4][0]), int(xyImagePoints_2[4][1])), (255, 150, 0), 2)
                    img = cv2.line(img, (int(xyImagePoints_2[7][0]), int(xyImagePoints_2[7][1])), (int(xyImagePoints_2[6][0]), int(xyImagePoints_2[6][1])), (255, 150, 0), 2)
                    img = cv2.polylines(img, [rect_front], True, (255, 0, 0), 2, -1)
                    cv2.fillPoly(overlay, pts=[rect_front], color=(255, 100, 0))
                    alpha = 0.2  # Transparency factor
                    img = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
                    img = cv2.circle(img, (int(xyImagePoints_2[0][0]), int(xyImagePoints_2[0][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[1][0]), int(xyImagePoints_2[1][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[2][0]), int(xyImagePoints_2[2][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[3][0]), int(xyImagePoints_2[3][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[4][0]), int(xyImagePoints_2[4][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[5][0]), int(xyImagePoints_2[5][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[6][0]), int(xyImagePoints_2[6][1])), radius=5, color=(255, 0, 0), thickness=-1)
                    img = cv2.circle(img, (int(xyImagePoints_2[7][0]), int(xyImagePoints_2[7][1])), radius=5, color=(255, 0, 0), thickness=-1)
                if showFrame:
                    img = cv2.putText(img, "Frame:" + str(i), (50, 1200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                if showTimestamp:
                    timestamp = str(list_images[i].timestamp) + "s"
                    img = cv2.putText(img, timestamp, (1650, 1200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    # img=cv2.putText(img,line_size,(960,1200),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
                list_images[i].img = img

        if showVideo:
            height_img, width_img, layers = img.shape
            size = (width_img, height_img)
            out = cv2.VideoWriter(os.path.join(pathSaveImage, name_file + ".m4v"), cv2.VideoWriter_fourcc(*'mp4v'), 30, size)
            for i in range(len(list_images)):
                out.write(list_images[i].img)
            out.release()
        if showImage:
            img = list_images[frame].img
            cv2.imshow(str(frame), img)
            cv2.waitKey(0)  # waits until a key is pressed
            cv2.destroyAllWindows()
        if saveImage:
            img = list_images[frame].img
            name_image = str(os.path.join(pathSaveImage, name_file + "_" + str(frame) + '.png'))
            cv2.imwrite(name_image, img)
        print("done")
