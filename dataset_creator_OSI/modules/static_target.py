from osi3.osi_datarecording_pb2 import SensorDataSeries
import sys


def static_target(path, width, length, height,  x, y, roll, pitch, yaw, PATH_TO_OSI):
    sys.path.insert(1, PATH_TO_OSI)
    sensor_data_series = SensorDataSeries()
    sensor_data_series.sensor_data.add()
    sensor_data = sensor_data_series.sensor_data[0]
    sensor_data.sensor_view.add()
    sensor_view = sensor_data.sensor_view[0]
    gt = sensor_view.global_ground_truth
    stationary_object = gt.stationary_object
    stationary_object.add()
    stationary_object[0].base.dimension.width = width
    stationary_object[0].base.dimension.length = length
    stationary_object[0].base.dimension.height = height
    stationary_object[0].base.position.x = x
    stationary_object[0].base.position.y = y
    stationary_object[0].base.orientation.roll = roll
    stationary_object[0].base.orientation.pitch = pitch
    stationary_object[0].base.orientation.yaw = yaw
    osi_trace_file = "obj_sv_350_300"  # name of the file
    with open(path + osi_trace_file + ".osi", "ab+") as f:  # open the osi file
        obj_ser = sensor_data_series.SerializeToString()  # serializing data
        f.write(obj_ser)
