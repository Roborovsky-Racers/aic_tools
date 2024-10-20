from aic_tools.common import map_to_world, lowpass_filter, m_per_sec_to_kmh
from aic_tools.topic_handler import GnssPoseHandler
from aic_tools.data_processor import (
    compute_gyro_odometry,
    compensate_gyro_odometry_by_ekf_localization,
)

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker


def plot_map_in_world(ax, map_data):
    map_left_bottom = map_to_world(
        0,
        map_data["size"][1],
        map_data["origin"],
        map_data["size"],
        map_data["resolution"],
    )
    map_right_top = map_to_world(
        map_data["size"][0],
        0,
        map_data["origin"],
        map_data["size"],
        map_data["resolution"],
    )
    extent = [
        map_left_bottom[0],
        map_right_top[0],
        map_left_bottom[1],
        map_right_top[1],
    ]

    ax.imshow(map_data["image_array"], cmap="gray", extent=extent)
    ax.set_xlim([extent[0], extent[1]])
    ax.set_ylim([extent[2], extent[3]])


def plot_reference_path(ax, ref_path_df):
    ax.plot(ref_path_df.x_m, ref_path_df.y_m, "--", label="reference path")


def plot_trajectory(
    ax,
    dataframes,
    df,
    t0=None,
    t1=None,
    tg=None,
    plot_gyro_odom=False,
    plot_gnss=False,
    plot_orientation=False,
    plot_velocity_text=False,
):
    if t0 is None:
        t0 = 0
    if t1 is None:
        t1 = len(df.stamp)

    # 軌跡をプロット
    ax.plot(df.ekf_x[t0:t1], df.ekf_y[t0:t1], label="ekf")

    if plot_gyro_odom:
        # ジャイロオドメトリを計算してdataframeに追加
        compute_gyro_odometry(df)

        # EKFローカリゼーションの位置を基準にジャイロオドメトリを補正する
        # reset_localization_indicesはEKFローカリゼーションの位置を基準にジャイロオドメトリを補正するためのリセットポイントのインデックス
        reset_localization_indices = []
        # reset_localization_indices = [3000, 4600, 6000, 7300, 9100]
        # reset_localization_indices = [3000, 4600, 6000, 9100]
        reset_points = compensate_gyro_odometry_by_ekf_localization(
            df, reset_localization_indices
        )

        ax.plot(df.gyro_odom_x[t0:t1], df.gyro_odom_y[t0:t1], label="gyro odom")

        if len(reset_points) > 0:
            ax.plot(
                [p[1] for p in reset_points],
                [p[2] for p in reset_points],
                "*",
                markersize=5.0,
                label="reset point",
            )

    if plot_gnss:
        # ax.plot(df.gnss_x[t0:t1], df.gnss_y[t0:t1], 'o', markersize=1.0, label="gnss")
        gnss_df = dataframes[GnssPoseHandler.TOPIC_NAME]  # 補間前のGNSSデータ

        if tg is None:
            tg = len(gnss_df.stamp)

        ax.plot(
            gnss_df.gnss_x[0:tg],
            gnss_df.gnss_y[0:tg],
            "o",
            markersize=2.0,
            label="gnss",
        )

    if plot_orientation:
        # 矢印で姿勢を表示 (ekf)
        skip = 50  # 矢印を飛ばしながら描画するためのステップサイズ
        ax.quiver(
            df.ekf_x[t0:t1:skip],
            df.ekf_y[t0:t1:skip],
            np.cos(df.ekf_yaw[t0:t1:skip]),
            np.sin(df.ekf_yaw[t0:t1:skip]),
            angles="xy",
            scale_units="xy",
            scale=1,
            color="blue",
            label="ekf yaw",
        )

        # 矢印で姿勢を表示 (gyro odom)
        ax.quiver(
            df.gyro_odom_x[t0:t1:skip],
            df.gyro_odom_y[t0:t1:skip],
            np.cos(df.gyro_odom_yaw[t0:t1:skip]),
            np.sin(df.gyro_odom_yaw[t0:t1:skip]),
            angles="xy",
            scale_units="xy",
            scale=1,
            color="red",
            label="gyro odom yaw",
        )

    if plot_velocity_text:
        skip = 100
        for i in range(t0, t1, skip):
            ax.text(
                df.ekf_x[i],
                df.ekf_y[i],
                f"{m_per_sec_to_kmh(df.vx[i]):.1f}",
                fontsize=6,
                color="blue",
            )

    ax.legend()
    ax.grid()
    plt.gca().set_aspect("equal", adjustable="box")


def plot_velocity_acceleration(dataframes, df):
    # compute acceleration
    dt = np.diff(df.stamp)
    acc_x = np.diff(df.vx) / dt
    acc_rz = np.diff(df.gyro_z) / dt

    # apply lowpass filter
    cutoff_frequency = 1.0
    sampling_rate = 1.0 / np.average(dt)
    acc_x = lowpass_filter(acc_x, cutoff_frequency, sampling_rate)
    acc_rz = lowpass_filter(acc_rz, cutoff_frequency, sampling_rate)

    fig, ax = plt.subplots(2, 1, figsize=(10, 6))

    ax[0].plot(df.stamp, df.vx, label="vx")
    ax[0].plot(df.stamp, df.gyro_z, label="gyro_z")
    ax[0].legend()
    ax[0].grid()
    ax[0].yaxis.set_major_locator(ticker.MultipleLocator(1.0))

    ax[1].plot(df.stamp[1:], acc_x, label="acc_x")
    # ax[1].plot(df.stamp[1:], acc_rz, label="acc_rz")
    ax[1].legend()
    ax[1].yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax[1].yaxis.set_minor_locator(ticker.MultipleLocator(0.25))
    ax[1].grid(True, which="both")

    plt.show()
