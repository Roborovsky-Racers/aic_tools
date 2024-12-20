#pragma once

#include <cstddef>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

namespace aic_tools {

class KartDiag : public rclcpp::Node {

public:
  /* alias */
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Bool = std_msgs::msg::Bool;
  using Empty = std_msgs::msg::Empty;
  using Float64 = std_msgs::msg::Float64;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using VelReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

public:
  KartDiag();

private:
bool mpc_controller_alive_{false};

// Subscribers
rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
rclcpp::Subscription<Bool>::SharedPtr sub_gnss_duplication_, sub_gnss_outlier_, sub_gnss_cov_large_, sub_ekf_cov_large_;
rclcpp::Subscription<SteeringReport>::SharedPtr sub_steering_report_;
rclcpp::Subscription<VelReport>::SharedPtr sub_vel_status_;

// Publishers
rclcpp::Publisher<Bool>::SharedPtr pub_gnss_data_lost_alert_;
rclcpp::Publisher<Bool>::SharedPtr pub_steer_rate_limit_alert_;
rclcpp::Publisher<Float64>::SharedPtr pub_estimated_acc_;

// Duplication detection
size_t gnss_duplication_count_;
std::unordered_map<size_t, size_t> gnss_duplication_count_map_;
rclcpp::Time last_gnss_time_{0, 0};

// Outlier detection
size_t gnss_outlier_count_;

// Covariance large alert
size_t gnss_cov_large_count_;
size_t ekf_cov_large_count_;

// steering rate limit alert
double filtered_angle_;
double last_filtered_angle_;
rclcpp::Time last_steering_time_{0, 0};

// acc estimation
double filtered_speed_;
double last_filtered_speed_;
rclcpp::Time last_speed_time_{0, 0};

// Parameters
size_t alert_gnss_duplication_count_;
double alert_gnss_data_lost_sec_;
std::string gnss_filter_node_name_;
double steering_rate_alert_threshold_;
double steering_rate_lpf_alpha_;
double acc_lpf_alpha_;

// Timer
rclcpp::TimerBase::SharedPtr timer_;
void on_timer();

void reset();

void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg);
void gnss_duplication_callback(const Bool::SharedPtr msg);
void gnss_outlier_callback(const Bool::SharedPtr msg);
void gnss_cov_large_callback(const Bool::SharedPtr msg);
void ekf_cov_large_callback(const Bool::SharedPtr msg);
void steer_status_callback(const SteeringReport::SharedPtr msg);
void vel_status_callback(const VelReport::SharedPtr msg);
void on_shutdown();
void update_duplicate_count_map();
void print_stats();


};

} // namespace aic_tools