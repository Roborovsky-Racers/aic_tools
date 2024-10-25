#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path
import matplotlib.pyplot as plt

import rclpy

from aic_tools.topic_handler import (
    TopicHandlerRegistry,
    LocalizationHandler,
    VelocityStatusHandler,
    SteeringStatusHandler,
    ImuDataHandler,
    GnssPoseHandler,
    AckermannCommandHandler,
    ActuationCommandHandler,
)
from aic_tools.rosbag_converter import convert_bag_to_csv, load_csv
from aic_tools.config_loader import try_load_mpc_config
from aic_tools.data_loader import load_reference_path, load_occupancy_grid_map
from aic_tools.data_processor import (
    interpolate_dataframes,
)
from aic_tools.data_plotter import (
    plot_map_in_world,
    plot_reference_path,
    plot_trajectory,
    plot_velocity_acceleration,
    plot_steer,
)


def parse_args(argv):
    (OCCUPANCY_GRID_MAP_YAML_PATH, REFERENCE_PATH_CSV_PATH) = try_load_mpc_config()

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "input_bag_directory",
        type=str,
        help="Directory where bag files (*.db3 and metadata.yaml) are stored",
    )
    parser.add_argument(
        "-r",
        "--reference_path_csv_path",
        type=str,
        help="Path to the reference path csv file",
        default=REFERENCE_PATH_CSV_PATH,
    )
    parser.add_argument(
        "-m",
        "--map_yaml_path",
        type=str,
        help="Path to the occupancy grid map yaml file",
        default=OCCUPANCY_GRID_MAP_YAML_PATH,
    )
    parser.add_argument(
        "-w",
        "--overwrite",
        action="store_true",
        help="Overwrite existing csv files",
    )
    parser.add_argument(
        "-s",
        "--save_plot",
        action="store_true",
        help="Save the plot as a png file",
    )

    args = parser.parse_args(argv)
    return args


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)  # type: ignore
    args = parse_args(args_without_ros[1:])

    TARGET_HANDLERS = [
        LocalizationHandler,
        VelocityStatusHandler,
        SteeringStatusHandler,
        ImuDataHandler,
        GnssPoseHandler,
        AckermannCommandHandler,
        ActuationCommandHandler,
    ]

    # 指定された HANDLERS をアクティブにする
    TopicHandlerRegistry.activate_handlers(TARGET_HANDLERS)
    active_handlers = TopicHandlerRegistry.get_active_handlers()

    # rosbag を csv に変換して書き出す
    # (overwrite == Falseの場合は、すでに csv が存在する場合は何もしない)
    convert_bag_to_csv(args.input_bag_directory, active_handlers, args.overwrite)

    # 書き出したcsvを読み込み、dataframeに格納する
    dataframes = load_csv(args.input_bag_directory, active_handlers)

    # dataframes を最も長い時系列を持つデータの時刻を基準に補間して1つのdataframeにまとめる
    df = interpolate_dataframes(dataframes)

    # プロット
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))

    if args.map_yaml_path != "":
        plot_map_in_world(ax, load_occupancy_grid_map(args.map_yaml_path))
    if args.reference_path_csv_path != "":
        plot_reference_path(ax, load_reference_path(args.reference_path_csv_path))

    plot_trajectory(
        ax,
        dataframes,
        df,
        # t_start=0.0,
        # t_end=70.0,
        plot_gyro_odom=False,
        plot_gnss=True,
        plot_orientation=False,
        plot_velocity_text=True,
    )

    if args.save_plot:
        # save the plot as a png file
        bag_name = Path(args.input_bag_directory).name
        save_path = "/aichallenge/workspace/png/" + bag_name + ".png"
        print("save_path: ", save_path)
        plt.savefig(save_path)
    else:
        plt.show()

    # plot_velocity_acceleration(dataframes, df)
    # plot_steer(df)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
