#!/usr/bin/env python3
import rclpy
import rosbag2_py


def trim_bag(input_bag: str, output_bag: str, start_time: float, end_time: float):
    storage_options = rosbag2_py.StorageOptions(
        uri=input_bag,
        storage_id='sqlite3'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag, storage_id='sqlite3'),
        converter_options
    )

    topics_info = reader.get_all_topics_and_types()
    writer.create_topic(topics_info)

    start_time_ns = int(start_time * 1e9)
    end_time_ns = int(end_time * 1e9)

    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()

        # 指定した開始時刻と終了時刻の範囲内でのみ書き込み
        if start_time_ns <= timestamp <= end_time_ns:
            writer.write(topic, data, timestamp)

if __name__ == "__main__":
    rclpy.init()

    # 入力bagファイル、出力bagファイル、開始時間(秒)、終了時間(秒)
    INPUT_BAG_DIRECTORY = "/aichallenge/workspace/0920_demo_interface_only"
    OUTPUT_BAG_DIRECTORY = "/aichallenge/workspace/test"

    trim_bag(INPUT_BAG_DIRECTORY, OUTPUT_BAG_DIRECTORY, start_time=10.0, end_time=20.0)

    rclpy.shutdown()
