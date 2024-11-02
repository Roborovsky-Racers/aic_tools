#include "aic_tools/kart_diag.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

namespace aic_tools {

KartDiag::KartDiag() : GnssFilter("kart_diag") {
  alert_gnss_duplicate_count_ =
      static_cast<size_t>(declare_parameter<int64_t>("alert_gnss_duplicate_count"));
  alert_gnss_discontinuation_sec_ =
      declare_parameter<double>("alert_gnss_discontinuation_sec");
  rclcpp::on_shutdown([this]() { this->on_shutdown(); });

}

void KartDiag::gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
  if (last_gnss_time_.seconds() != 0) {
    const auto diff_time = rclcpp::Time{msg->header.stamp} - last_gnss_time_;
    if (diff_time.seconds() > alert_gnss_discontinuation_sec_) {
      RCLCPP_WARN_STREAM(get_logger(), "GNSS time gap: " << diff_time.seconds() << " sec");
    }
  }
  last_gnss_time_ = msg->header.stamp;

  if (is_duplicate(*msg)) {
    gnss_duplicate_count_++;
    if (gnss_duplicate_count_ >= alert_gnss_duplicate_count_) {
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "GNSS duplication count: " << gnss_duplicate_count_);
    }
  }
  else{
    update_duplicate_count_map();
    gnss_duplicate_count_ = 0;
  }

  // push unique data to queue
  if (gnss_queue_.size() >= gnss_queue_size_) {
    gnss_queue_.pop_front();
  }
  gnss_queue_.push_back(*msg);

  if (is_outlier(*msg))
  {
    RCLCPP_WARN(get_logger(), "GNSS outlier detected");
  }

}

void KartDiag::update_duplicate_count_map() {
  if (gnss_duplicate_count_ > 0)
  {
    if (gnss_duplicate_count_map_.find(gnss_duplicate_count_) == gnss_duplicate_count_map_.end())
    {
      gnss_duplicate_count_map_[gnss_duplicate_count_] = 1;
    }
    else
    {
      gnss_duplicate_count_map_[gnss_duplicate_count_]++;
    }
  }
}

void KartDiag::print_stats() {
  // GNSS Value Duplication
  std::map<size_t, size_t> sorted_duplicate_count_map(gnss_duplicate_count_map_.begin(), gnss_duplicate_count_map_.end());
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
  rclcpp::spin(std::make_shared<aic_tools::KartDiag>());
  rclcpp::shutdown();
  return 0;
}
