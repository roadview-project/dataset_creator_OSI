import sys
from rosbag.bag import Bag
import yaml
from collections import namedtuple


def main():
    bag = Bag(sys.argv[1])
    topics = bag.get_type_and_topic_info()[1]
    sensors = {}

    for topic in topics:
        message_type = topics[topic].msg_type
        if input(f"For Topic {topic} with message_type {message_type} \n do you want to add this to the dataset? ")[0].lower() == "n":
            continue
        function_from_type_name = check_message_type(message_type)
        name, sensor_config = function_from_type_name()

        sensors[name] = sensor_config | {"topic": topic}

    with open("config.yaml", "w+") as file:
        file.write(yaml.dump(sensors))


def check_message_type(message_type):
    if message_type == "sensor_msgs/CompressedImage":
        return camera
    elif message_type == "sensor_msgs/PointCloud2":
        return point_cloud
    else:
        return other


def get_mounting_position():
    x = float(input("Insert the X mounting Position: "))
    y = float(input("Insert the Y mounting Position: "))
    z = float(input("Insert the Z mounting Position: "))
    roll = float(input("Insert the roll mounting Position: "))
    pitch = float(input("Insert the pitch mounting Position: "))
    yaw = float(input("Insert the yaw mounting Position: "))
    return {"mounting_position": {"x": x,
                                  "y": y,
                                  "z": z,
                                  "roll": roll,
                                  "pitch": pitch,
                                  "yaw": yaw}}


def camera():
    print("a Camera was indentified;")
    name = input("Insert the name of this sensor: ")
    image_resolution_horizontal = int(
        input("Insert the number of pixels horizontal: "))
    image_resolution_vertical = int(
        input("Insert the number of pixels vertical: "))
    print(f"regarding the {name}´s mounting Position")
    mounting_position = get_mounting_position()

    return (name, {"sensor_type": "Camera",
                   "image_resolution_horizontal": image_resolution_horizontal,
                   "image_resolution_vertical": image_resolution_vertical} | mounting_position)


def point_cloud():
    print("a PointCloud-based sensor was indentified;")
    name = input("Insert the name of this sensor: ")
    # this will later link to the function call
    # so write the same name as the function call
    sensor_type = input("Insert this sensor type(Lidar or Radar): ")
    sensor_type = sensor_type.lower()
    sensor_type = sensor_type[0].upper() + sensor_type[1:]
    mounting_position = get_mounting_position()
    return (name, {"sensor_type": sensor_type} | mounting_position)


def other():
    print("This is a stub for you to qualify any other sensors please add")
    name = input("Insert the name of this sensor: ")
    mounting_position = get_mounting_position()
    return (name, mounting_position)


if __name__ == '__main__':
    main()
