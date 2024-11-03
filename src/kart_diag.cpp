#include "aic_tools/kart_diag.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/clock.hpp>

namespace aic_tools {

KartDiag::KartDiag() : Node("kart_diag") {
  alert_gnss_duplication_count_ =
      static_cast<size_t>(declare_parameter<int64_t>("alert_gnss_duplication_count"));
  alert_gnss_data_lost_sec_ =
      declare_parameter<double>("alert_gnss_data_lost_sec");

  // Subscribers
  sub_gnss_ = this->create_subscription<PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&KartDiag::gnss_callback, this, std::placeholders::_1));
  sub_gnss_duplication_ = this->create_subscription<Bool>(
        "/gnss_filter/is_gnss_duplication_detected",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&KartDiag::gnss_duplication_callback, this, std::placeholders::_1));
  sub_gnss_outlier_ = this->create_subscription<Bool>(
        "/gnss_filter/is_gnss_outlier_detected",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&KartDiag::gnss_outlier_callback, this, std::placeholders::_1));
  sub_gnss_cov_large_ = this->create_subscription<Bool>(
        "/gnss_filter/is_gnss_cov_large",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&KartDiag::gnss_cov_large_callback, this, std::placeholders::_1));
  sub_ekf_cov_large_ = this->create_subscription<Bool>(
        "/gnss_filter/is_ekf_cov_large",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&KartDiag::ekf_cov_large_callback, this, std::placeholders::_1));
  sub_mpc_dead_signal_ = this->create_subscription<Empty>(
        "/mpc/dead_signal",
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        std::bind(&KartDiag::mpc_dead_signal_callback, this, std::placeholders::_1));

  // Publishers
  pub_gnss_data_lost_alert_ = this->create_publisher<Bool>(
      "~/is_gnss_lost", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

  rclcpp::on_shutdown([this]() { this->on_shutdown(); });

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&KartDiag::on_timer, this));
}

void KartDiag::reset()
{
  gnss_duplication_count_ = 0;
  gnss_duplication_count_map_.clear();
  last_gnss_time_ = rclcpp::Time(0, 0);
}

void KartDiag::gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
  last_gnss_time_ = get_clock()->now();
}

void KartDiag::gnss_duplication_callback(const Bool::SharedPtr msg) {
  if (!msg->data) {
    update_duplicate_count_map();
    gnss_duplication_count_ = 0;
    return;
  }

  gnss_duplication_count_++;
  if (gnss_duplication_count_ >= alert_gnss_duplication_count_) {
    RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "GNSS duplication count: " << gnss_duplication_count_);
  }
}

void KartDiag::gnss_outlier_callback(const Bool::SharedPtr msg) {
  if (msg->data) {
    RCLCPP_WARN(get_logger(), "GNSS Outlier detected!");
  }
}

void KartDiag::gnss_cov_large_callback(const Bool::SharedPtr msg) {
  if (msg->data) {
    RCLCPP_WARN(get_logger(), "GNSS covariance is too large!");
  }
}

void KartDiag::ekf_cov_large_callback(const Bool::SharedPtr msg) {
  if (msg->data) {
    RCLCPP_WARN(get_logger(), "EKF covariance is too large!");
  }
}

void KartDiag::mpc_dead_signal_callback(const Empty::SharedPtr msg) {
  RCLCPP_WARN(get_logger(), "MPC dead signal detected!");
  print_stats();
  reset();
}

void KartDiag::on_timer() {
  Bool alert_msg;
  if (last_gnss_time_.seconds() == 0.0) {
    return;
  }
  const auto lost_duration = get_clock()->now() - last_gnss_time_;
  if (lost_duration > rclcpp::Duration::from_seconds(alert_gnss_data_lost_sec_)) {
    alert_msg.data = true;
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 500, "GNSS data lost for " << lost_duration.seconds() << " seconds");
  }
  else {
    alert_msg.data = false;
  }
  pub_gnss_data_lost_alert_->publish(alert_msg);
}

void KartDiag::update_duplicate_count_map() {
  if (gnss_duplication_count_ > 0)
  {
    if (gnss_duplication_count_map_.find(gnss_duplication_count_) == gnss_duplication_count_map_.end())
    {
      gnss_duplication_count_map_[gnss_duplication_count_] = 1;
    }
    else
    {
      gnss_duplication_count_map_[gnss_duplication_count_]++;
    }
  }
}

void KartDiag::print_stats() {
  // GNSS Value Duplication
  std::map<size_t, size_t> sorted_duplicate_count_map(gnss_duplication_count_map_.begin(), gnss_duplication_count_map_.end());
  std::ostringstream oss;
  oss << "\n--------------------";
  oss << "\nGNSS duplication count: frequency";
  for (const auto &pair : sorted_duplicate_count_map) {
    oss << "\n  " << pair.first << " times: " << pair.second;
  }
  RCLCPP_INFO_STREAM(get_logger(), oss.str());
}

void KartDiag::on_shutdown() {
  print_stats();
}

} // namespace aic_tools

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<aic_tools::KartDiag>();
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  exec->add_node(node);
  exec->spin();
  rclcpp::shutdown();
  return 0;
}
