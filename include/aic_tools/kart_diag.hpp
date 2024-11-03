#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace aic_tools {

class KartDiag : public rclcpp::Node {

public:
  /* alias */
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using Bool = std_msgs::msg::Bool;
  using Empty = std_msgs::msg::Empty;

public:
  KartDiag();

private:

// Subscribers
rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
rclcpp::Subscription<Bool>::SharedPtr sub_gnss_duplication_, sub_gnss_outlier_, sub_gnss_cov_large_, sub_ekf_cov_large_;
rclcpp::Subscription<Empty>::SharedPtr sub_mpc_dead_signal_;

// Publishers
rclcpp::Publisher<Bool>::SharedPtr pub_gnss_data_lost_alert_;


// Duplication detection
size_t gnss_duplication_count_;
std::unordered_map<size_t, size_t> gnss_duplication_count_map_;
rclcpp::Time last_gnss_time_{0, 0};

// Parameters
size_t alert_gnss_duplication_count_;
double alert_gnss_data_lost_sec_;

// Timer
rclcpp::TimerBase::SharedPtr timer_;
void on_timer();

void reset();

void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg);
void gnss_duplication_callback(const Bool::SharedPtr msg);
void gnss_outlier_callback(const Bool::SharedPtr msg);
void gnss_cov_large_callback(const Bool::SharedPtr msg);
void ekf_cov_large_callback(const Bool::SharedPtr msg);
void mpc_dead_signal_callback(const Empty::SharedPtr msg);
void on_shutdown();
void update_duplicate_count_map();
void print_stats();


};

} // namespace aic_tools