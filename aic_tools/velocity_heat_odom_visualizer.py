#!/usr/bin/env python3

import numpy as np
import copy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import ColorRGBA

class VelocityHeatOdomVisualizer(Node):

    MIN_VELOCITY = 2.77 # m/s (10 km/h)
    # MAX_VELOCITY = 8.3 # m/s (30 km/h)
    MAX_VELOCITY = 6.94 # m/s (25 km/h)
    PUB_RATE = 2.0 # Hz

    def __init__(self) -> None:
        super().__init__("velocity_heat_odom_visualizer")

        self._markers = Marker()
        self._setup_pub_sub()
        self._last_publish_time = self.get_clock().now()
        self._marker_id = 0

    def _setup_pub_sub(self) -> None:
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(
            Marker, "/vel_heat_odom_marker", latching_qos)

        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)

    def _odom_callback(self, msg: Odometry) -> None:
        self._publish_velocity_heat_odom(msg)

    def _publish_velocity_heat_odom(self, msg: Odometry) -> None:
        if (self.get_clock().now() - self._last_publish_time).nanoseconds / 1e9 < 1.0 / self.PUB_RATE:
            return
        self._last_publish_time = self.get_clock().now()
        velocity_norm = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        text = Marker()
        text.header.frame_id = "map"
        text.ns = f"vel_heat_odom_text_{self._marker_id}"
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = copy.deepcopy(msg.pose.pose)
        text.pose.position.x += 1.0
        text.pose.position.y += 1.0
        text.pose.position.z = 101.0
        text.scale.z = 1.0
        text.text = f"{velocity_norm * 3.6:.2f}"
        text.color = self.create_vel_heat_color(velocity_norm)

        self._marker_pub.publish(text)
        self._marker_id += 1

    @classmethod
    def create_vel_heat_color(cls, vel_norm: float, alpha: float = 1.0) -> ColorRGBA:
        normalized_velocity = max(
            0.0,
            min(
                (vel_norm - cls.MIN_VELOCITY) / (cls.MAX_VELOCITY - cls.MIN_VELOCITY),
                1.0)
            )
        r = normalized_velocity
        g = 0.2
        b = 1.0 - normalized_velocity
        return ColorRGBA(r=r, g=g, b=b, a=alpha)

