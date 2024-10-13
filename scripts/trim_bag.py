#!/usr/bin/env python3
import rclpy
from aic_tools.bag_tools import trim_bag


if __name__ == "__main__":
    rclpy.init()

    # 入力bagファイル、出力bagファイル、開始時間(秒)、終了時間(秒)
    INPUT_BAG_DIRECTORY = "/aichallenge/workspace/0920_demo_interface_only"
    OUTPUT_BAG_DIRECTORY = "/aichallenge/workspace/test"

    trim_bag(INPUT_BAG_DIRECTORY, OUTPUT_BAG_DIRECTORY, start_time=10.0, end_time=20.0)

    rclpy.shutdown()
