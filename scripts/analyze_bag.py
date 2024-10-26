#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path
import matplotlib.pyplot as plt

import rclpy

from aic_tools.topic_handler import (
    TopicHandlerRegistry,
    LocalizationHandler,
    LocalizationAccelerationHandler,
    VelocityStatusHandler,
    SteeringStatusHandler,
    ImuDataHandler,
    GnssPoseHandler,
    AckermannCommandHandler,
    ActuationCommandHandler,
)
from aic_tools.rosbag_converter import convert_bag_to_csv, load_csv
from aic_tools.config_loader import try_load_mpc_config
from aic_tools.data_processor import (
    interpolate_dataframes,
)
from aic_tools.data_plotter import (
    set_save_base_name,
    save_plot,
    plot_map_in_world,
    plot_reference_path,
    plot_trajectory,
    plot_velocity_acceleration,
    plot_steer,
    plot_gnss_covariance,
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

    bag_name = Path(args.input_bag_directory).name
    set_save_base_name("/aichallenge/workspace/png/" + bag_name)

    TARGET_HANDLERS = [
        LocalizationHandler,
        LocalizationAccelerationHandler,
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

    plot_trajectory(
        dataframes,
        df,
        # t_start=0.0,
        # t_end=50.0,
        plot_gyro_odom=False,
        plot_gnss=True,
        plot_orientation=False,
        plot_velocity_text=True,
        map_yaml_path=args.map_yaml_path,
        reference_path_csv_path=args.reference_path_csv_path,
        save=args.save_plot,
    )

    # plot_velocity_acceleration(dataframes, df, save=args.save_plot)
    # plot_velocity_acceleration(dataframes, df, t_start=10, t_end=14, plot_acc=False, save=args.save_plot)
    # plot_velocity_acceleration(dataframes, df, t_start=115, t_end=120, plot_acc=False, save=args.save_plot)

    # plot_steer(df, save=args.save_plot)
    # plot_steer(df, t_start=10, t_end=12, save=args.save_plot)

    # plot_gnss_covariance(dataframes, save=args.save_plot)
    # plot_gnss_covariance(dataframes, t_start=1470, t_end=1630, save=args.save_plot)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
