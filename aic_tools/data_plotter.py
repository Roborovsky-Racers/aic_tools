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
    t_start=0.0,
    t_end=None,
    plot_gyro_odom=False,
    plot_gnss=False,
    plot_orientation=False,
    plot_velocity_text=False,
    quiver_skip_duration = 1.0,
    velocity_skip_duration = 1.0,
):
    ave_dt = df.stamp.diff().mean()

    t0 = df[df.stamp >= t_start].index[0]
    t1 = len(df.stamp)
    if t_end is not None:
        t1 = df[df.stamp <= t_end].index[-1]

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
        gnss_df.stamp = (gnss_df.stamp - gnss_df.stamp[0]) / 1e9

        tg0 = gnss_df[gnss_df.stamp >= t_start].index[0]
        tg1 = len(gnss_df.stamp)
        if t_end is not None:
            tg1 = gnss_df[gnss_df.stamp <= t_end].index[-1]

        ax.plot(
            gnss_df.gnss_x[tg0:tg1],
            gnss_df.gnss_y[tg0:tg1],
            "o",
            markersize=2.0,
            label="gnss",
        )

    if plot_orientation:
        # 矢印で姿勢を表示 (ekf)
        quiver_skip = int(quiver_skip_duration / ave_dt)
        ax.quiver(
            df.ekf_x[t0:t1:quiver_skip],
            df.ekf_y[t0:t1:quiver_skip],
            np.cos(df.ekf_yaw[t0:t1:quiver_skip]),
            np.sin(df.ekf_yaw[t0:t1:quiver_skip]),
            angles="xy",
            scale_units="xy",
            scale=1,
            color="blue",
            label="ekf yaw",
        )

        # 矢印で姿勢を表示 (gyro odom)
        if plot_gyro_odom:
            ax.quiver(
                df.gyro_odom_x[t0:t1:quiver_skip],
                df.gyro_odom_y[t0:t1:quiver_skip],
                np.cos(df.gyro_odom_yaw[t0:t1:quiver_skip]),
                np.sin(df.gyro_odom_yaw[t0:t1:quiver_skip]),
                angles="xy",
                scale_units="xy",
                scale=1,
                color="red",
                label="gyro odom yaw",
            )

    if plot_velocity_text:
        velocity_skip = int(velocity_skip_duration / ave_dt)
        for i in range(t0, t1, velocity_skip):
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

def plot_steer(df, T=None):
    if T is None:
        T=len(df.stamp)

    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    ax.plot(df.stamp[0:T], df.steering_tire_angle_command[0:T], label="steer_cmd")
    # ax.plot(df.stamp[0:T], df.actuation_steer_cmd[0:T], label="actuator_steer_cmd")
    ax.plot(df.stamp[0:T], df.steer[0:T], label="steer")
    # ax.plot(df.stamp, df.steer * 1.639, label="steer * 1.639")
    # ax.plot(df.stamp[0:T], df.steer[0:T] / 1.639, label="steer / 1.639")
    ax.set_ylim([-0.6, 0.6])
    ax.legend()
    ax.grid()
    # plt.savefig("sim_steer.png")
    # plt.savefig("real_steer.png")
    plt.show()