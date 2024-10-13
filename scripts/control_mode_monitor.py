#!/usr/bin/env python3
from typing import List
import pandas as pd
import rclpy
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import ControlModeReport
from aic_tools.bag_tools import trim_bag


class AutonomousModePeriodMarker(Node):
    def __init__(self) -> None:
        super().__init__("autonomous_mode_period_marker")

        self.create_subscription(
            ControlModeReport,
            "vehicle/status/control_mode",
            self._control_mode_callback,
            10
        )
        self._last_mode = None
        self.auto_periods = pd.DataFrame(columns=['start', 'end'])
        self._current_start_time = None

    def _control_mode_callback(self, msg):
        if self._last_mode is None:
            # First message received
            self._last_mode = msg.mode
            if msg.mode == ControlModeReport.AUTONOMOUS:
                # We are in AUTONOMOUS mode already, start period
                self._current_start_time = self.time_msg_to_sec(msg.stamp)
            return

        if self._last_mode != msg.mode:
            # Mode has changed
            if msg.mode == ControlModeReport.AUTONOMOUS:
                print("Manual -> Autonomous")
                self._current_start_time = self.time_msg_to_sec(msg.stamp)
            elif msg.mode == ControlModeReport.MANUAL:
                print("Autonomous -> Manual")
                end_time = self.time_msg_to_sec(msg.stamp)
                if self._current_start_time is not None:
                    # Append the period
                    new_period = {'start': self._current_start_time, 'end': end_time}
                    self.auto_periods = self.auto_periods.append(new_period, ignore_index=True)
                    self._current_start_time = None
                else:
                    # End received before start
                    new_period = {'start': float('nan'), 'end': end_time}
                    self.auto_periods = self.auto_periods.append(new_period, ignore_index=True)
            self._last_mode = msg.mode

    def dump_auto_period_as_csv(self):
        self.auto_periods.to_csv('auto_period.csv', index=False)

    def trim_bag(self):
        for i in range(len(self.auto_periods)):
            trim_bag(
                "/aichallenge/workspace/rosbag2_autoware/rosbag2_2024_10_11-18_03_34/rosbag2_2024_10_11-18_03_34_0.db3",
                f"/aichallenge/workspace/rosbag2_autoware/rosbag2_2024_10_11-18_03_34/rosbag2_2024_10_11-18_03_34_0_{i}.db3",
                self.auto_periods['start'][i],self.auto_periods['end'][i])

    @staticmethod
    def time_msg_to_sec(msg) -> float:
        return msg.sec + msg.nanosec * 1e-9


if __name__ == "__main__":
    rclpy.init()

    node = AutonomousModePeriodMarker()

    try:
        rclpy.spin(node)
    finally:
        # If still in AUTONOMOUS mode, close the last period with the current time
        if node._current_start_time is not None:
            now = node.get_clock().now().to_msg()
            end_time = node.time_msg_to_sec(now)
            new_period = {'start': node._current_start_time, 'end': end_time}
            node.auto_periods = node.auto_periods.append(new_period, ignore_index=True)
            node._current_start_time = None
        print(node.auto_periods)
        node.dump_auto_period_as_csv()
        node.trim_bag()
        node.destroy_node()
        rclpy.shutdown()
