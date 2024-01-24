import sys
import os
import re
import yaml
import logging
import numpy as np
from rosbag.bag import Bag

import importlib
from tqdm.auto import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm


def main():
    """
    creates dataset from rosbags
    usage:
    python3 dataset_creator.py PATH_TO_CONFIG ENTIRE_PATH_TO_BAG 
    python dataset_creator.py config.yaml /home/leoneto/open-simulation-interface temp/ /media/DATA/05_carissma_outdoor_week18_23/01_pedestrian/01_day/01_clear_weather/01_28m/0deg.bag
    """
    assert len(
        sys.argv) == 6, "wrong argument number check: python3 dataset_creator.py CONFIG path to targets PATH_TO_OSI save_file_path bag_location"

    config_location = sys.argv[1]
    path_to_target = sys.argv[2]
    PATH_TO_OSI = sys.argv[3]
    save_file_path = sys.argv[4]
    bag_location = sys.argv[5]

    logger = create_logger("dataset_creator")
    logger.info(f"Now working on {bag_location} with config {config_location}")
    logger.setLevel(logging.DEBUG)
    with open(config_location, "r")as handle:
        config = yaml.safe_load(handle)

    logger.info(f"Loaded Config with {len(config.keys())} sensors")

    sensor_classes = instantiate_classes(config, save_file_path, PATH_TO_OSI)

    bag = Bag(bag_location)

    create_target(bag_location, save_file_path, PATH_TO_OSI, path_to_target)

    read_bag(bag, config, sensor_classes, logger)

    logger.info(f"finished {bag_location}")


def create_target(bag_location, file_path, PATH_TO_OSI, path_to_target):
    from dataset_creator_OSI.modules.static_target import static_target
    with open(path_to_target) as handle:
        targets = yaml.safe_load(handle)["targets"]

    bag_name = re.split("/|_|\.", bag_location)
    mask_target = np.isin(bag_name, list(targets.keys()))
    distances_RTK = {28: [-47.62, -59.17], 56: [-62.82, -82.74],
                     84: [-77.9,  -106.15], 112: [-93.18, -129.74]}
    distances = np.array(["28m", "56m", "84m", "112m"])
    mask_distances = np.isin(distances, bag_name)
    angles = np.array(["0deg", "45deg", "90deg"])
    mask_angles = np.isin(angles, bag_name)

    if np.any(mask_target):
        distance = int(distances[mask_distances][0].split("m")[0])
        x, y = distances_RTK[distance]
        yaw = int(angles[mask_angles][0].split("deg")[0]
                  ) if angles[mask_angles].size > 0 else 0
        target_type = np.array(bag_name)[mask_target][0]
        target = targets[target_type]
        target_file_path = file_path+"/"+target_type.upper()+"_TARGET/"
        os.makedirs(target_file_path, exist_ok=True)
        static_target(target_file_path, target["width"], target["length"],
                      target["height"],  x, y, 0, 0, yaw, PATH_TO_OSI)


def instantiate_classes(config, file_path, PATH_TO_OSI):
    sensor_classes = {}
    for key in config:
        sensor_file_path = file_path+"/"+key+"/"
        os.makedirs(sensor_file_path, exist_ok=True)
        sensor_type = config[key]["sensor_type"]
        SensorClass = getattr(importlib.import_module(
            "dataset_creator_OSI.modules."+sensor_type), sensor_type)
        sensor_classes[config[key]["topic"]] = SensorClass(
            sensor_file_path, config[key], PATH_TO_OSI)
    return sensor_classes


def create_logger(name):
    console_formatter = logging.Formatter(
        "%(asctime)s | %(levelname)s | %(message)s")
    console_logger = logging.StreamHandler(sys.stdout)
    console_logger.setFormatter(console_formatter)
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    logger.addHandler(console_logger)
    logger.info("logger created")
    dup_filter = DuplicateFilter()
    logger.addFilter(dup_filter)
    return logger


""" Begin Snippet from https://stackoverflow.com/questions/31953272/logging-print-message-only-once by chepner"""


class DuplicateFilter(object):
    def __init__(self):
        self.msgs = set()

    def filter(self, record):
        rv = record.msg not in self.msgs
        self.msgs.add(record.msg)
        return rv


"""End Snippet """


def read_bag(bag, config, sensor_classes, logger):
    topics = [config[i]["topic"] for i in config]
    logger.info("Reading Messages")
    with logging_redirect_tqdm([logger]):
        for topic, msg, t in tqdm(bag.read_messages(), total=bag.get_message_count(), desc=bag._file.name):
            if topic not in topics:
                logger.info(f"Message {topic} ignored")
                continue
            sensor_classes[topic].export(msg)
            logger.info(f"Topic {topic} worked with")
    logger.info("Messages Read")


if __name__ == '__main__':
    main()
