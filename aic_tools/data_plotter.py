from aic_tools.common import map_to_world, lowpass_filter, m_per_sec_to_kmh
from aic_tools.topic_handler import (
    GnssPoseHandler,
    LocalizationHandler,
)
from aic_tools.data_loader import load_reference_path, load_occupancy_grid_map
from aic_tools.data_processor import (
    compute_gyro_odometry,
    compensate_gyro_odometry_by_ekf_localization,
)

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

SAVE_BASE_NAME = "./output_plots"


def set_save_base_name(save_base_name):
    global SAVE_BASE_NAME
    SAVE_BASE_NAME = save_base_name


def save_plot(fig, name, save=False):
    global SAVE_BASE_NAME
    if save:
        save_path = f"{SAVE_BASE_NAME}_{name}.png"
        print(f"Saving plot to {save_path}")
        fig.savefig(save_path)


def get_index_from_time(df, t_start=None, t_end=None):
    if df.stamp[0] != 0:
        df.stamp = (df.stamp - df.stamp[0]) / 1e9

    t0 = 0
    t1 = len(df.stamp)
    if t_start is not None:
        t0 = df[df.stamp >= t_start].index[0]
    if t_end is not None:
        t1 = df[df.stamp <= t_end].index[-1]
    return t0, t1


def get_interval(df, duration: float) -> int:
    ave_dt = df.stamp.diff().mean()
    return int(duration / ave_dt)


def interactive_compute_slope(ax):
    while True:
        # 2点をクリックして座標を取得
        points = plt.ginput(2, timeout=-1)

        # 終了条件（点が選択されなかった場合）
        if len(points) < 2:
            print("Exiting...")
            break

        # 座標を取得して傾きを計算
        x1, y1 = points[0]
        x2, y2 = points[1]
        if x2 != x1:
            slope = (y2 - y1) / (x2 - x1)
            slope_text_str = f"Slope: {slope:.2f}"
        else:
            slope_text_str = "Slope: undefined"

        # 新しい直線をプロット
        (line,) = ax.plot([x1, x2], [y1, y2], "r--", linewidth=2)

        # 傾きのテキストをプロット
        slope_text = ax.text(
            (x1 + x2) / 2,
            (y1 + y2) / 2,
            slope_text_str,
            color="blue",
            fontsize=10,
            ha="center",
        )

        # グラフを更新
        plt.draw()


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
    dataframes,
    df,
    t_start=None,
    t_end=None,
    plot_gyro_odom=False,
    plot_gnss=False,
    plot_orientation=False,
    plot_velocity_text=False,
    quiver_skip_duration=1.0,
    velocity_skip_duration=1.0,
    map_yaml_path="",
    reference_path_csv_path="",
    save=False,
):
    fig, ax = plt.subplots(1, 1, figsize=(16, 10))

    if map_yaml_path != "":
        plot_map_in_world(ax, load_occupancy_grid_map(map_yaml_path))
    if reference_path_csv_path != "":
        plot_reference_path(ax, load_reference_path(reference_path_csv_path))

    t0, t1 = get_index_from_time(df, t_start, t_end)

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
        tg0, tg1 = get_index_from_time(gnss_df, t_start, t_end)

        ax.plot(
            gnss_df.gnss_x[tg0:tg1],
            gnss_df.gnss_y[tg0:tg1],
            "o",
            markersize=2.0,
            label="gnss",
        )

    if plot_orientation:
        # 矢印で姿勢を表示 (ekf)
        quiver_skip = get_interval(df, quiver_skip_duration)
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
        velocity_skip = get_interval(df, velocity_skip_duration)
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

    if save:
        save_plot(fig, f"trajectory", save)
    else:
        plt.show()

    plt.clf()
    plt.close()


def plot_velocity_acceleration(
    dataframes, df, t_start=None, t_end=None, plot_acc=True, save=False
):
    t0, t1 = get_index_from_time(df, t_start, t_end)

    # compute acceleration
    dt = np.diff(df.stamp[t0:t1])
    acc_x = np.diff(df.vx[t0:t1]) / dt
    acc_rz = np.diff(df.gyro_z[t0:t1]) / dt

    # apply lowpass filter
    cutoff_frequency = 1.0
    sampling_rate = 1.0 / np.average(dt)
    acc_x = lowpass_filter(acc_x, cutoff_frequency, sampling_rate)
    acc_rz = lowpass_filter(acc_rz, cutoff_frequency, sampling_rate)

    if plot_acc:
        fig, ax = plt.subplots(2, 1, figsize=(10, 6))
    else:
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        ax = [ax]

    ax[0].plot(df.stamp[t0:t1], df.vx[t0:t1], label="vx")
    # ax[0].plot(df.stamp, df.gyro_z, label="gyro_z")
    ax[0].plot(df.stamp[t0:t1], df.acceleration_command[t0:t1], label="accel cmd")
    # ax[0].plot(df.stamp[t0:t1], df.actuation_accel_cmd[t0:t1], label="accel pedal")
    # ax[0].plot(df.stamp[t0:t1], -df.actuation_brake_cmd[t0:t1], label="brake_pedal")
    # ax[0].plot(df.stamp[t0:t1], df.steer[t0:t1], label="steer")
    ax[0].set_ylim([-2.0, 10.0])
    ax[0].legend()
    ax[0].xaxis.set_major_locator(ticker.MultipleLocator(10.0))
    # ax[0].xaxis.set_minor_locator(ticker.MultipleLocator(2.0))
    # ax[0].yaxis.set_major_locator(ticker.MultipleLocator(1.0))
    # ax[0].xaxis.set_major_locator(ticker.MultipleLocator(1.0))
    # ax[0].xaxis.set_minor_locator(ticker.MultipleLocator(0.2))
    # ax[0].yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    # ax[0].yaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    ax[0].grid(True, which="both")

    if plot_acc:
        ax[1].plot(df.stamp[t0 + 1 : t1], acc_x, label="acc_x")
        ax[1].plot(df.stamp[t0:t1], df.loc_acc_x[t0:t1], label="loc_acc_x")
        ax[1].legend()
        ax[1].yaxis.set_major_locator(ticker.MultipleLocator(1.0))
        # ax[1].yaxis.set_minor_locator(ticker.MultipleLocator(0.25))
        ax[1].grid(True, which="both")

    interactive_compute_slope(ax[0])
    save_plot(fig, f"velocity_acceleration_{t0}_{t1}", save)
    plt.clf()
    plt.close()


def plot_steer(df, t_start=None, t_end=None, save=False):
    t0, t1 = get_index_from_time(df, t_start, t_end)

    fig, ax = plt.subplots(1, 1, figsize=(16, 10))
    ax.plot(df.stamp[t0:t1], df.steering_tire_angle_command[t0:t1], label="steer_cmd")
    # ax.plot(df.stamp[t0:t1], df.actuation_steer_cmd[t0:t1], label="actuator_steer_cmd")
    ax.plot(df.stamp[t0:t1], df.steer[t0:t1], label="steer")
    # ax.plot(df.stamp, df.steer * 1.639, label="steer * 1.639")
    # ax.plot(df.stamp[t0:t1], df.steer[t0:t1] / 1.639, label="steer / 1.639")
    ax.set_ylim([-0.6, 0.6])
    ax.legend()
    ax.grid()
    interactive_compute_slope(ax)
    save_plot(fig, f"steer_{t0}_{t1}", save)
    plt.show()
    plt.clf()
    plt.close()


def plot_gnss_covariance(dataframes, t_start=None, t_end=None, save=False):
    df = dataframes[GnssPoseHandler.TOPIC_NAME]  # 補間前のGNSSデータ
    t0, t1 = get_index_from_time(df, t_start, t_end)

    x_centered = df.gnss_x[t0:t1] - df.gnss_x[t0:t1].mean()
    y_centered = df.gnss_y[t0:t1] - df.gnss_y[t0:t1].mean()
    z_centered = df.gnss_z[t0:t1] - df.gnss_z[t0:t1].mean()

    gnss_x_std = x_centered.std()
    gnss_y_std = y_centered.std()
    gnss_z_std = z_centered.std()
    print(
        f"gnss_x_std: {gnss_x_std}, gnss_y_std: {gnss_y_std}, gnss_z_std: {gnss_z_std}"
    )

    fig, ax = plt.subplots(1, 2, figsize=(10, 6))
    ax[0].plot(
        df.stamp[t0:t1], x_centered, "o", markersize=1.0, label="gnss_x_centered"
    )
    ax[0].plot(
        df.stamp[t0:t1], y_centered, "o", markersize=1.0, label="gnss_y_centered"
    )
    # ax[0].plot(df.stamp[t0:t1], z_centered, "o", markersize=1.0, label="gnss_z_centered")
    ax[0].legend()
    ax[0].grid()
    ax[0].set_ylim([-0.04, 0.04])
    ax[0].set_xlabel("time [s]")
    ax[0].set_ylabel("position [m]")

    ax[1].plot(x_centered, y_centered, "o", markersize=1.0, label="gnss_centered")
    ax[1].legend()
    ax[1].grid()
    ax[1].set_xlim([-0.04, 0.04])
    ax[1].set_ylim([-0.04, 0.04])
    ax[1].set_xlabel("x [m]")
    ax[1].set_ylabel("y [m]")
    fig.gca().set_aspect("equal", adjustable="box")

    plt.show()
    save_plot(fig, f"gnss_covariance_{t0}_{t1}", save)
    plt.clf()
    plt.close()


def plot_gnss_and_ekf_with_phase_diff(dataframes, phase_diff=0.0, t_start=None, t_end=None, save=False):

    gnss_df = dataframes[GnssPoseHandler.TOPIC_NAME]
    gnss_df.stamp = (gnss_df.stamp - gnss_df.stamp[0]) / 1e9

    gnss_x = gnss_df.gnss_x.drop_duplicates()
    gnss_stamp = gnss_df.stamp[gnss_x.index]
    vel_gnss_x = gnss_x.diff() / gnss_stamp.diff()

    ekf_df = dataframes[LocalizationHandler.TOPIC_NAME]
    ekf_df.stamp = (ekf_df.stamp - ekf_df.stamp[0]) / 1e9
    vel_ekf_x = ekf_df.ekf_x.diff() / ekf_df.stamp.diff()

    t0, t1 = get_index_from_time(ekf_df, t_start, t_end)

    fig, ax = plt.subplots(1, 1, figsize=(16, 10))

    # ax.plot(ekf_df.stamp[t0:t1], vel_ekf_x[t0:t1], label="ekf")
    ax.plot(ekf_df.stamp[t0:t1], ekf_df.ekf_x[t0:t1], label="ekf")

    gnss_df.stamp = gnss_df.stamp[gnss_x.index].reset_index(drop=True)
    t0, t1 = get_index_from_time(gnss_df, t_start, t_end)
    gnss_df.stamp = gnss_df.stamp - phase_diff
    # ax.plot(gnss_df.stamp[t0:t1], vel_gnss_x[t0:t1], label="gnss")
    ax.plot(gnss_df.stamp[t0:t1], gnss_x[t0:t1], label="gnss")

    plt.grid()
    plt.legend()
    plt.show()
    save_plot(fig, f"gnss_and_ekf_with_phase_diff_{t0}_{t1}_phase_diff_{phase_diff}", save)
    plt.clf()
    plt.close()