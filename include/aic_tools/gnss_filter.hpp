#pragma once

#include "rclcpp/node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <angles/angles.h>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <cmath>
#include <deque>

namespace aic_tools {

using Float64 = std_msgs::msg::Float64;
using PoseWithCovarianceStamped =
    geometry_msgs::msg::PoseWithCovarianceStamped;
using Odometry = nav_msgs::msg::Odometry;

class GnssFilter : public rclcpp::Node {

public:
  GnssFilter();
  GnssFilter(const std::string & node_name);

protected:
  bool is_duplicate(const PoseWithCovarianceStamped &p0,
                    const PoseWithCovarianceStamped &p1);
  bool is_duplicate(const PoseWithCovarianceStamped &msg);
  bool is_outlier(const PoseWithCovarianceStamped &msg);

  virtual void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg);

  std::deque<PoseWithCovarianceStamped> gnss_queue_;

  // parameters
  size_t gnss_queue_size_;

private:
  void init();

  void ekf_odom_callback(const Odometry::SharedPtr msg);

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Publisher<Float64>::SharedPtr pub_distance_;
  rclcpp::Publisher<Float64>::SharedPtr pub_yaw_, pub_yaw_diff_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_ekf_odom_;

  // parameters
  double ekf_keep_duration_;
  double outlier_threshold_;
  double gnss_delay_sec_;
  bool enable_duplicate_detection_;
  bool enable_outlier_detection_;

  // runtime states
  double last_yaw_ = 0.0;
  double last_ekf_yaw_ = 0.0;

  std::deque<Odometry> ekf_odom_queue_;

};

} // namespace aic_tools