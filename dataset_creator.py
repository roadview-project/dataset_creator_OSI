import sys
import os
import re
import argparse
import yaml
import logging
import numpy as np
from rosbag.bag import Bag

import importlib
from tqdm.auto import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm

from osi3.osi_groundtruth_pb2 import GroundTruth

from dataset_creator_OSI.exporter import MCAPExporter


def main():
    """Create an OSI MCAP dataset from a rosbag.

    Usage:
        python3 dataset_creator.py CONFIG targets.yaml save_path bag_location [--include-raw]
    """
    args = parse_args()

    logger = create_logger("dataset_creator")
    logger.info(f"Now working on {args.bag_location} with config {args.config}")
    logger.setLevel(logging.DEBUG)

    with open(args.config, "r") as handle:
        config = yaml.safe_load(handle)

    logger.info(f"Loaded Config with {len(config.keys())} sensors")

    sensor_classes = instantiate_classes(config)

    bag = Bag(args.bag_location)

    # Build output filename with _multi_ type code per OSI spec
    bag_name = os.path.splitext(os.path.basename(args.bag_location))[0]
    output_path = os.path.join(args.save_path, f"{bag_name}_multi.mcap")
    os.makedirs(args.save_path, exist_ok=True)

    description = f"Converted from {os.path.basename(args.bag_location)}"

    with MCAPExporter(output_path, include_raw=args.include_raw,
                      description=description) as exporter:
        # Register OSI channels for each sensor
        for sensor_name in config:
            topic = config[sensor_name]["topic"]
            sensor = sensor_classes[topic]
            exporter.add_osi_channel(sensor_name, sensor.MESSAGE_TYPE)

        # Register raw ROS channels if requested
        if args.include_raw:
            topic_info = bag.get_type_and_topic_info()[1]
            for sensor_name in config:
                topic = config[sensor_name]["topic"]
                if topic in topic_info:
                    exporter.add_raw_channel(topic, topic_info[topic].msg_type)

        # Register and write target channel
        target_msg = create_target(args.bag_location, args.targets)
        if target_msg is not None:
            exporter.add_osi_channel("ground_truth_target", GroundTruth)
            exporter.write_osi_message("ground_truth_target", target_msg)

        # Process bag messages
        read_bag(bag, config, sensor_classes, exporter, logger)

    logger.info(f"Finished {args.bag_location} -> {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert rosbag sensor recordings to OSI MCAP trace files.")
    parser.add_argument("config", help="Path to sensor config YAML")
    parser.add_argument("targets", help="Path to targets YAML")
    parser.add_argument("save_path", help="Output directory for MCAP files")
    parser.add_argument("bag_location", help="Path to input rosbag")
    parser.add_argument("--include-raw", action="store_true",
                        help="Embed original ROS messages alongside OSI channels")
    return parser.parse_args()


def create_target(bag_location, path_to_target):
    """Build a GroundTruth message for the static target based on bag path.

    Returns:
        A GroundTruth message, or None if no target matches.
    """
    from dataset_creator_OSI.modules.static_target import build_static_target

    with open(path_to_target) as handle:
        targets = yaml.safe_load(handle)["targets"]

    bag_name = re.split("/|_|\\.", bag_location)
    mask_target = np.isin(bag_name, list(targets.keys()))
    distances_RTK = {28: [-47.62, -59.17], 56: [-62.82, -82.74],
                     84: [-77.9, -106.15], 112: [-93.18, -129.74]}
    distances = np.array(["28m", "56m", "84m", "112m"])
    mask_distances = np.isin(distances, bag_name)
    angles = np.array(["0deg", "45deg", "90deg"])
    mask_angles = np.isin(angles, bag_name)

    if np.any(mask_target):
        if not np.any(mask_distances):
            return None
        distance = int(distances[mask_distances][0].split("m")[0])
        if distance not in distances_RTK:
            return None
        x, y = distances_RTK[distance]
        yaw = int(angles[mask_angles][0].split("deg")[0]
                  ) if angles[mask_angles].size > 0 else 0
        target_type = np.array(bag_name)[mask_target][0]
        target = targets[target_type]
        return build_static_target(
            target["width"], target["length"], target["height"],
            x, y, 0, 0, yaw)
    return None


def instantiate_classes(config):
    """Dynamically load sensor classes from config.

    Returns:
        dict mapping ROS topic -> sensor instance.
    """
    sensor_classes = {}
    for key in config:
        sensor_type = config[key]["sensor_type"]
        SensorClass = getattr(importlib.import_module(
            "dataset_creator_OSI.modules." + sensor_type), sensor_type)
        sensor_classes[config[key]["topic"]] = SensorClass(config[key])
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


def read_bag(bag, config, sensor_classes, exporter, logger):
    """Read bag messages and write to MCAP via the exporter."""
    topics = [config[i]["topic"] for i in config]
    topic_to_sensor_name = {config[k]["topic"]: k for k in config}

    logger.info("Reading Messages")
    with logging_redirect_tqdm([logger]):
        for topic, msg, t in tqdm(bag.read_messages(), total=bag.get_message_count(), desc=bag._file.name):
            if topic not in topics:
                logger.info(f"Message {topic} ignored")
                continue

            sensor_name = topic_to_sensor_name[topic]
            sensor = sensor_classes[topic]

            # Convert ROS message to OSI protobuf
            osi_msg = sensor.export(msg)
            exporter.write_osi_message(sensor_name, osi_msg)

            # Optionally write raw ROS message
            if exporter.include_raw:
                raw_data = msg._buff if hasattr(msg, '_buff') else msg.serialize_numpy() if hasattr(msg, 'serialize_numpy') else b""
                if raw_data:
                    timestamp_ns = int(t.to_nsec())
                    exporter.write_raw_message(topic, raw_data, timestamp_ns)

            logger.info(f"Topic {topic} worked with")
    logger.info("Messages Read")


if __name__ == '__main__':
    main()
