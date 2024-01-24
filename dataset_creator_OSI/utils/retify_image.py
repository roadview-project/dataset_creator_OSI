from osi3.osi_datarecording_pb2 import SensorDataSeries
from osi3.osi_sensordata_pb2 import SensorData
import google.protobuf.message
import google.protobuf.json_format
import numpy as np
import cv2
import sys
import struct
import math as m
import json
import base64
import os
import pathlib
sys.path.insert(1, '/home/leoneto/open-simulation-interface')


def folder_json(path_osi_file):
    path = pathlib.PurePath(path_osi_file)
    list_elements = list(path.parts)
    list_elements[3] = "TWICE_json"
    json_name = list_elements[-1] + "_undistorded"
    list_elements.pop()
    path_json = os.path.join(*list_elements) + "/"
    if os.path.exists(path_json):
        "The directory already exists"
    else:
        os.makedirs(path_json)
    return(path_json, json_name)


def merge_JsonFiles(files, timestr):
    with open(timestr, "w") as outfile:
        outfile.write('{}'.format('\n'.join([open(f, "r").read() for f in files])))
    # Function to repair merged json file


def fix_json(timestr, timestr_json):
    # Read in the file
    with open(timestr, 'r') as file:
        filedata = file.read()

    # Replace the target string
    filedata = filedata.replace('}\n{', ',')

    # Write the file out again
    with open(timestr_json, 'w') as file:
        file.write(filedata)


# this function open a osi file and exports the image and the time stamp to .json
def img_osi_json(path_osi):
    list_json = []
    img = SensorData()
    n_img = 0
    json_path, json_name = folder_json(path_osi)
    camera_matrix = np.array([[1822.81074890076, 0, 892.638558402846], [0, 2014.07112980207, 750.855378319723], [0, 0, 1]])
    dist_coeffs = np.array([-0.331266119932407, 0.196579423555906, 0, 0, 0])
    width = 1920
    height = 1208
    newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (width, height), 1, (width, height))
    with open(path_osi, 'rb') as f:
        while 1:
            message_size_bytes = f.read(struct.calcsize("<L"))
            if len(message_size_bytes) == 0:
                break
            message_size = struct.unpack("<L", message_size_bytes)[0]
            message_bytes = f.read(message_size)
            img.ParseFromString(message_bytes)
            str_msg = img.sensor_view[0].camera_sensor_view[0].image_data
            cv_img = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=str_msg)
            im = cv2.imdecode(cv_img, cv2.IMREAD_ANYCOLOR) 
            undistorded_image = cv2.undistort(im, camera_matrix, dist_coeffs, None, newcameramatrix)
            image_bytes = cv2.imencode(".jpg", undistorded_image)[1].tobytes()
            info = img.sensor_view[0].camera_sensor_view[0].view_configuration
            base_64_img = base64.b64encode(image_bytes)
            img_string = base_64_img.decode('utf-8')
            dict_img = {n_img: {
                "image_data": img_string,

                'Stamp': {
                    "Sec": img.sensor_view[0].timestamp.seconds,
                    "Nsec": img.sensor_view[0].timestamp.nanos

                }
            }
            }
            json_data = json.dumps(dict_img, indent=2)
            name_json = '/home/leoneto/img_json_temp/' + str(n_img) + ".json"
            list_json.append(name_json)
            with open(name_json, 'w') as json_file:
                json_file.write(json_data)
            n_img += 1
    # merge json files to export to matlab
    name_of_file = json_path + json_name
    merge_JsonFiles(list_json, name_of_file + ".txt")
    fix_json(name_of_file + '.txt', name_of_file + ".json")
    # removing temporary archives
    os.remove(name_of_file + ".txt")
    for arquivo in list_json:
        os.remove(arquivo)


path_camera_osi = "/media/DATA/TWICE/Scenarios/static_ego/pedestrian/synthetic/daytime/cluster/test_run_1/Camera_DDI/camera_sv_350_300.osi"
parent = os.path.dirname(os.path.dirname(path_camera_osi))
path_cam_ota = parent + "/Camera_OTA"
cam_ota_osi = path_cam_ota + "/camera_sv_350_300.osi"
img_osi_json(cam_ota_osi)
