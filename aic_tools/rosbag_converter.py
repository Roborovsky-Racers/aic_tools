from pathlib import Path
import pandas as pd
import csv

from rclpy.serialization import deserialize_message

import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from aic_tools.topic_handler import TopicHandlerRegistry


def read_messages(input_bag: Path, topics: list[str]):

    storage_options = rosbag2_py.StorageOptions(
        uri=str(input_bag), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic not in topics:
            continue

        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp

    del reader


def convert_bag_to_csv(
    input_bag_directory: str,
    active_handlers: list[TopicHandlerRegistry],
    overwrite: bool,
):

    csv_paths = [
        Path(input_bag_directory) / f"{handler.get_under_scored_topic_name()}.csv"
        for handler in active_handlers
    ]
    if (not overwrite) and all([csv_path.exists() for csv_path in csv_paths]):
        print("csv files already exist")
        return

    print("converting bag to csv")

    active_topics = [handler.get_topic_name() for handler in active_handlers]

    csv_files = {
        topic: open(csv_path, "w") for topic, csv_path in zip(active_topics, csv_paths)
    }
    csv_writers = {}

    def write_to_csv(topic, d: dict):
        if topic not in csv_writers:
            csv_writers[topic] = csv.DictWriter(csv_files[topic], fieldnames=d.keys())
            csv_writers[topic].writeheader()
        csv_writers[topic].writerow(d)

    for topic, msg, timestamp in read_messages(input_bag_directory, active_topics):
        handler = TopicHandlerRegistry.get_handler(topic)
        if handler:
            write_to_csv(topic, handler.process_message(msg, timestamp))


def load_csv(base_path: str, active_handler: list[TopicHandlerRegistry]):
    dataframes = {}

    for handler in active_handler:
        csv_path = Path(base_path) / f"{handler.get_under_scored_topic_name()}.csv"
        if not csv_path.exists():
            print(f"{csv_path} not found")
            return

        dataframes[handler.get_topic_name()] = pd.read_csv(csv_path)
        # print(dataframes[name].dtypes)

    return dataframes
