import pandas as pd
import numpy as np
from scipy.interpolate import interp1d


def interpolate_dataframes(dataframes: dict):
    # 最も長いstampを基準とする
    longest_key = max(dataframes, key=lambda key: len(dataframes[key]["stamp"]))
    longest_df = dataframes[longest_key]
    reference_stamps = longest_df["stamp"].values

    # 各DataFrameを基準のstampに補間
    interpolated_dfs = []

    for key, df in dataframes.items():
        # 'stamp' 列を取り出す
        original_stamps = df["stamp"].values

        # 補間するための関数を作成（データ列に対して適用）
        interpolated_df = pd.DataFrame({"stamp": reference_stamps})

        for column in df.columns:
            if column != "stamp":  # 'stamp' 列は補間しない
                # 補間関数を作成
                f = interp1d(
                    original_stamps,
                    df[column].values,
                    bounds_error=False,
                    fill_value="extrapolate",
                )
                # 補間を実行し、基準のstampに合わせる
                interpolated_df[column] = f(reference_stamps)

        interpolated_dfs.append(interpolated_df)

    # 補間されたデータフレームを結合
    combined_df = pd.concat(
        interpolated_dfs, axis=1, join="inner"
    )  # 共通のstampを基準に結合
    combined_df = combined_df.loc[
        :, ~combined_df.columns.duplicated()
    ]  # 重複した列を削除

    # stampの一番始めの値を0として、経過時間に変換し、さらにナノ秒から秒に変換
    combined_df.stamp = (
        combined_df.stamp - combined_df.stamp[0]
    ) / 1e9  # ナノ秒から秒に変換

    return combined_df


def compute_gyro_odometry(df):
    # 初期状態をnumpy配列に変換
    gyro_odom_x = np.zeros(len(df))
    gyro_odom_y = np.zeros(len(df))
    gyro_odom_yaw = np.zeros(len(df))

    # 初期値を設定
    gyro_odom_x[0] = df.ekf_x[0]
    gyro_odom_y[0] = df.ekf_y[0]
    gyro_odom_yaw[0] = df.ekf_yaw[0]

    # 時間差分 (dt) をベクトル計算
    dt = np.diff(df.stamp)  # 時間差分を計算
    dt = np.insert(dt, 0, 0)  # 最初のステップのdtを0に設定

    # 速度 (v) と角速度 (omega) をベクトルで取得
    v = df.vx.values
    omega = df.gyro_z.values

    # ヨー角の変化をベクトルで計算
    gyro_odom_yaw[1:] = gyro_odom_yaw[0] + np.cumsum(omega[1:] * dt[1:])

    # X, Y の変化をベクトルで計算
    delta_x = v * np.cos(gyro_odom_yaw) * dt
    delta_y = v * np.sin(gyro_odom_yaw) * dt

    # 初期位置からの積算を計算
    gyro_odom_x[1:] = gyro_odom_x[0] + np.cumsum(delta_x[1:])
    gyro_odom_y[1:] = gyro_odom_y[0] + np.cumsum(delta_y[1:])

    # 結果をDataFrameに追加
    df["gyro_odom_x"] = gyro_odom_x
    df["gyro_odom_y"] = gyro_odom_y
    df["gyro_odom_yaw"] = gyro_odom_yaw


def compensate_gyro_odometry_by_ekf_localization(
    df, reset_localization_indices: list[int]
):
    diff_x = np.diff(df.gyro_odom_x)
    diff_y = np.diff(df.gyro_odom_y)
    diff_yaw = np.diff(df.gyro_odom_yaw)

    reset_points = []

    for i in reset_localization_indices:
        df.gyro_odom_x[i] = df.ekf_x[i]
        df.gyro_odom_x[i + 1 :] = df.gyro_odom_x[i] + np.cumsum(diff_x[i:])
        df.gyro_odom_y[i] = df.ekf_y[i]
        df.gyro_odom_y[i + 1 :] = df.gyro_odom_y[i] + np.cumsum(diff_y[i:])
        df.gyro_odom_yaw[i] = df.ekf_yaw[i]
        df.gyro_odom_yaw[i + 1 :] = df.gyro_odom_yaw[i] + np.cumsum(diff_yaw[i:])

        reset_points.append((df.stamp[i], df.ekf_x[i], df.ekf_y[i], df.ekf_yaw[i]))

    return reset_points
