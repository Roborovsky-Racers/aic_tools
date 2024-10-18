#!/usr/bin/env python3
import rclpy
from pathlib import Path

import matplotlib.pyplot as plt

from aic_tools.topic_handler import (
    TopicHandlerRegistry,
    LocalizationHandler,
    VelocityStatusHandler,
    SteeringStatusHandler,
    ImuDataHandler,
    GnssPoseHandler,
)

from aic_tools.rosbag_converter import convert_bag_to_csv, load_csv
from aic_tools.data_loader import load_reference_path, load_occupancy_grid_map
from aic_tools.data_processor import (
    interpolate_dataframes,
    compute_gyro_odometry,
    compensate_gyro_odometry_by_ekf_localization,
)
from aic_tools.data_plotter import (
    plot_map_in_world,
    plot_reference_path,
    plot_trajectory,
    plot_velocity_acceleration,
)


def main():
    rclpy.init()

    INPUT_BAG_DIRECTORY = Path(
        "/logs/20241011_training/rosbag_trim/rosbag2_2024_10_11-18_03_34_trim_12/"
        # "/logs/20241011_training/rosbag_trim/rosbag2_2024_10_11-18_03_34_trim_7/"
    )
    TARGET_HANDLERS = [
        LocalizationHandler,
        VelocityStatusHandler,
        SteeringStatusHandler,
        ImuDataHandler,
        GnssPoseHandler,
    ]

    REFERENCE_PATH_CSV_PATH = Path(
        "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/env/final_ver2/traj_mincurv.csv"
    )
    OCCUPANCY_GRID_MAP_YAML_PATH = Path(
        "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/env/final_ver2/occupancy_grid_map.yaml"
    )

    # 指定された HANDLERS をアクティブにする
    TopicHandlerRegistry.activate_handlers(TARGET_HANDLERS)
    active_handlers = TopicHandlerRegistry.get_active_handlers()

    # rosbag を csv に変換して書き出す
    # (すでに csv が存在する場合は何もしない)
    convert_bag_to_csv(INPUT_BAG_DIRECTORY, active_handlers)

    # 書き出したcsvを読み込み、dataframeに格納する
    dataframes = load_csv(INPUT_BAG_DIRECTORY, active_handlers)

    # dataframes を最も長い時系列を持つデータの時刻を基準に補間して1つのdataframeにまとめる
    df = interpolate_dataframes(dataframes)

    # ジャイロオドメトリを計算してdataframeに追加
    compute_gyro_odometry(df)

    # EKFローカリゼーションの位置を基準にジャイロオドメトリを補正する
    # reset_localization_indicesはEKFローカリゼーションの位置を基準にジャイロオドメトリを補正するためのリセットポイントのインデックス
    reset_localization_indices = []
    reset_localization_indices = [3000, 4600, 6000, 7300, 9100]
    # reset_localization_indices = [3000, 4600, 6000, 9100]
    reset_points = compensate_gyro_odometry_by_ekf_localization(
        df, reset_localization_indices
    )

    # プロット
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    plot_map_in_world(ax, load_occupancy_grid_map(OCCUPANCY_GRID_MAP_YAML_PATH))
    plot_reference_path(ax, load_reference_path(REFERENCE_PATH_CSV_PATH))
    plot_trajectory(
        ax,
        dataframes,
        df,
        reset_points,
        t0=0,
        t1=14000,
        # t0=0,
        # t1=14000,
        tg=1200,
        plot_gnss=True,
        plot_orientation=False,
        plot_velocity_text=True,
    )

    ax.legend()
    ax.grid()
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()

    # plot_velocity_acceleration(dataframes, df)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
