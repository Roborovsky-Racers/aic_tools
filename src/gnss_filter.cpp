#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

#include "rclcpp/logging.hpp"
#include <angles/angles.h>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <limits>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <cmath>
#include <deque>
#include <ranges>

namespace aic_tools {

class GnssFilter : public rclcpp::Node {
public:
  using Bool = std_msgs::msg::Bool;
  using Float64 = std_msgs::msg::Float64;
  using Int64 = std_msgs::msg::Int64;
  using PoseWithCovarianceStamped =
      geometry_msgs::msg::PoseWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;

  GnssFilter() : Node("gnss_filter") {
    // load parameters
    gnss_queue_size_ =
        static_cast<size_t>(declare_parameter<int64_t>("gnss_queue_size"));
    ekf_queue_size_ = static_cast<size_t>(declare_parameter<int64_t>("ekf_queue_size"));
    default_gnss_pose_covariance_ = declare_parameter<double>("default_gnss_pose_covariance");
    gnss_pose_covariance_elapsed_scaling_factor_ = declare_parameter<double>("gnss_pose_covariance_elapsed_scaling_factor");
    standard_gnss_publish_period_ = declare_parameter<double>("standard_gnss_publish_period");
    outlier_detection_angle_threshold_ = declare_parameter<double>("outlier_detection_angle_threshold");
    outlier_detection_distance_threshold_ = declare_parameter<double>("outlier_detection_distance_threshold");
    gnss_too_large_cov_threshold_ = declare_parameter<double>("gnss_too_large_cov_threshold");
    ekf_too_large_cov_threshold_ = declare_parameter<double>("ekf_too_large_cov_threshold");
    enable_duplicate_detection_ =
        declare_parameter<bool>("enable_duplicate_detection");
    enable_outlier_detection_ =
        declare_parameter<bool>("enable_outlier_detection");

    // create pub/sub
    pub_gnss_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance_filtered",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    pub_angle_between_gnss_and_ekf_ = this->create_publisher<Float64>(
        "~/angle_between_gnss_and_ekf", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_duplication_count_ = this->create_publisher<Int64>(
        "~/gnss_duplication_count", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_elapsed_from_last_unique_gnss_received_ = this->create_publisher<Float64>(
        "~/elapsed_from_last_unique_gnss_received", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_duplication_alert_ = this->create_publisher<Bool>(
        "~/is_gnss_duplication_detected", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_outlier_alert_ = this->create_publisher<Bool>(
        "~/is_gnss_outlier_detected", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_gnss_large_cov_alert_ = this->create_publisher<Bool>(
        "~/is_gnss_cov_large", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_ekf_large_cov_alert_ = this->create_publisher<Bool>(
        "~/is_ekf_cov_large", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    sub_gnss_ = this->create_subscription<PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&GnssFilter::gnss_callback, this, std::placeholders::_1));

    sub_ekf_odom_ = this->create_subscription<Odometry>(
        "/localization/kinematic_state",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        std::bind(&GnssFilter::ekf_odom_callback, this, std::placeholders::_1));
  }

private:
  void publish_duplication_count(const size_t count) {
    Int64 msg;
    msg.data = count;
    pub_duplication_count_->publish(msg);
  }

  void publish_elapsed_from_last_unique_gnss_received(const double elapsed) {
    Float64 msg;
    msg.data = elapsed;
    pub_elapsed_from_last_unique_gnss_received_->publish(msg);
  }

  void publish_duplication_alert(const bool is_duplicate) {
    Bool msg;
    msg.data = is_duplicate;
    pub_duplication_alert_->publish(msg);
  }

  void publish_gnss_large_cov_alert(const bool is_large_cov) {
    Bool msg;
    msg.data = is_large_cov;
    pub_gnss_large_cov_alert_->publish(msg);
  }

  void publish_ekf_large_cov_alert(const bool is_large_cov) {
    Bool msg;
    msg.data = is_large_cov;
    pub_ekf_large_cov_alert_->publish(msg);
  }

  void publish_outlier_alert(const bool is_outlier) {
    Bool msg;
    msg.data = is_outlier;
    pub_outlier_alert_->publish(msg);
  }

  void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
    // check duplicate 
    const bool is_duplicate_detected = is_duplicate(*msg);
    if (is_duplicate_detected) {
      ++duplication_count_;
      RCLCPP_DEBUG(get_logger(), "Detected duplicate message %zu times", duplication_count_);
    } else {
      duplication_count_ = 0;
    }

    // check large covariance
    const bool is_gnss_too_large_cov_detected = is_too_large_covariance(msg->pose.covariance[0], gnss_too_large_cov_threshold_);
    if (is_gnss_too_large_cov_detected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Too large GNSS pose covariance detected!");
    }

    // check outlier
    const bool is_outlier_detected = is_outlier(*msg, is_duplicate_detected);

    // compute elapsed time from the last unique gnss message
    const auto elapsed_from_last_unique_gnss_received = gnss_queue_.empty() ? 0.0 : compute_stamp_difference(gnss_queue_.back(), *msg);

    const bool is_gnss_lost = elapsed_from_last_unique_gnss_received > 1.0;

    // compute covariance
    static constexpr auto UNRELIABLE_COVARIANCE = 100000.0;
    const auto MAX_UNRELIABLE_GNSS_COUNT = static_cast<size_t>(2.0 / standard_gnss_publish_period_);
    if(is_gnss_too_large_cov_detected || is_outlier_detected || is_gnss_lost) {
      // Set high covariance if the covariance is too large or the message is outlier
      msg->pose.covariance[7*0] = UNRELIABLE_COVARIANCE;
      msg->pose.covariance[7*1] = UNRELIABLE_COVARIANCE;
      msg->pose.covariance[7*2] = UNRELIABLE_COVARIANCE;
      unreliable_gnss_count_ = MAX_UNRELIABLE_GNSS_COUNT;
    } else if(unreliable_gnss_count_ > 0) {
      // Decrease the covariance stepwise according to the unreliable gnss count
      --unreliable_gnss_count_;
      // const auto scale = 1.0 / std::pow(10.0, MAX_UNRELIABLE_GNSS_COUNT - unreliable_gnss_count_);
      const auto scale = std::pow(1.0 / UNRELIABLE_COVARIANCE, 
                               static_cast<double>(MAX_UNRELIABLE_GNSS_COUNT - unreliable_gnss_count_) 
                               / MAX_UNRELIABLE_GNSS_COUNT);
      const auto pose_cov = UNRELIABLE_COVARIANCE * scale;
      RCLCPP_WARN(get_logger(), "Decrease the covariance scale: %f, pose_cov: %f", scale, pose_cov);
      msg->pose.covariance[7*0] = pose_cov;
      msg->pose.covariance[7*1] = pose_cov;
      msg->pose.covariance[7*2] = pose_cov;
    }
    else {
      // Increase the covariance stepwise according to the elapsed time from the last unique gnss message
      double pose_cov = default_gnss_pose_covariance_;

      if(elapsed_from_last_unique_gnss_received > standard_gnss_publish_period_) {
          pose_cov = pose_cov
            * std::pow(gnss_pose_covariance_elapsed_scaling_factor_,
              elapsed_from_last_unique_gnss_received);
      }

      msg->pose.covariance[7*0] = pose_cov;
      msg->pose.covariance[7*1] = pose_cov;
      msg->pose.covariance[7*2] = pose_cov;
    }

    if(!is_duplicate_detected && !is_outlier_detected && !is_gnss_too_large_cov_detected) {
        // push unique message to the queue
        if (gnss_queue_.size() >= gnss_queue_size_) {
          gnss_queue_.pop_front();
        }
        gnss_queue_.push_back(*msg);

        // backup the last reliable gnss pose
        last_reliable_gnss_pose_ = gnss_queue_.back();
    }

    // publish the filtered message
    pub_gnss_->publish(*msg);
    RCLCPP_DEBUG(get_logger(), "Published filtered message");

    // publish alerts
    publish_duplication_count(duplication_count_);
    publish_elapsed_from_last_unique_gnss_received(elapsed_from_last_unique_gnss_received);
    publish_duplication_alert(is_duplicate_detected);
    publish_gnss_large_cov_alert(is_gnss_too_large_cov_detected);
    publish_outlier_alert(is_outlier_detected);
  }

  void ekf_odom_callback(const Odometry::SharedPtr msg) {
    if (ekf_odom_queue_.size() >= ekf_queue_size_) {
      ekf_odom_queue_.pop_front();
    }
    ekf_odom_queue_.push_back(*msg);

    const bool is_ekf_cov_large_detected = is_too_large_covariance(msg->pose.covariance[0], ekf_too_large_cov_threshold_);
    if (is_ekf_cov_large_detected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,  "Too large EKF pose covariance detected!");
    }

    publish_ekf_large_cov_alert(is_ekf_cov_large_detected);
  }

  double compute_stamp_difference(const PoseWithCovarianceStamped &p0,
                    const PoseWithCovarianceStamped &p1) {


    const auto & time0 = rclcpp::Time(p0.header.stamp.sec, p0.header.stamp.nanosec, RCL_ROS_TIME);
    const auto & time1 = rclcpp::Time(p1.header.stamp.sec, p1.header.stamp.nanosec, RCL_ROS_TIME);
    return (time1 - time0).seconds();
  }

  bool is_duplicate(const PoseWithCovarianceStamped &p0,
                    const PoseWithCovarianceStamped &p1) {
    return p0.pose.pose.position.x == p1.pose.pose.position.x &&
           p0.pose.pose.position.y == p1.pose.pose.position.y &&
           p0.pose.pose.position.z == p1.pose.pose.position.z;
  }

  bool is_duplicate(const PoseWithCovarianceStamped &msg) {
    if(!enable_duplicate_detection_) {
      return false;
    }

    for (const auto &p : gnss_queue_) {
      if (is_duplicate(p, msg)) {
        return true;
      }
    }
    return false;
  }

  bool is_too_large_covariance(const double covariance, const double too_large_cov_threshold) {
    return covariance > too_large_cov_threshold;
  }

  bool is_outlier(const PoseWithCovarianceStamped &msg, const bool is_duplicate_detected) {
    if(!enable_outlier_detection_) {
      return false;
    }

    if(is_duplicate_detected) {
      return false;
    }

    if(ekf_odom_queue_.empty()) {
      return false;
    }

    if(!last_reliable_gnss_pose_.has_value()) {
      return false;
    }

    const auto compute_position_diff_unit_vector = [](const geometry_msgs::msg::Pose &p0,
                                                 const geometry_msgs::msg::Pose &p1) {
      const double dx = p1.position.x - p0.position.x;
      const double dy = p1.position.y - p0.position.y;
      const double norm = std::sqrt(dx * dx + dy * dy);
      return std::make_tuple(Eigen::Vector2d(dx / norm, dy / norm), norm);
    };

    const auto get_nearest_stamp_ekf_pose = [&](const builtin_interfaces::msg::Time &time_msg, const bool pop_old) {

      const rclcpp::Time time(time_msg.sec, time_msg.nanosec, RCL_ROS_TIME);
      double min_diff_time = std::numeric_limits<double>::max();
      auto nearest_it = ekf_odom_queue_.begin();

      for(auto it = ekf_odom_queue_.begin(); it != ekf_odom_queue_.end(); ++it) {
        const auto &odom = *it;
        const double diff_time = std::abs((time - odom.header.stamp).seconds());

        if (diff_time > min_diff_time) {
          break;
        }

        min_diff_time = diff_time;
        nearest_it = it;
      }

      const Odometry nearest_ekf_pose = *nearest_it;

      if(pop_old && nearest_it != ekf_odom_queue_.begin()) {
        ekf_odom_queue_.erase(ekf_odom_queue_.begin(), nearest_it);
      }

      return nearest_ekf_pose.pose.pose;
    };

    const auto& last_reliable_gnss_pose = last_reliable_gnss_pose_.value();
    const auto& latest_gnss_pose = msg;

    const auto last_reliable_gnss_stamped_ekf_pose = get_nearest_stamp_ekf_pose(last_reliable_gnss_pose.header.stamp, true);
    const auto latest_gnss_stamped_ekf_pose = get_nearest_stamp_ekf_pose(latest_gnss_pose.header.stamp, false);

    const auto [gnss_diff_unit_vector, gnss_diff_norm] = compute_position_diff_unit_vector(last_reliable_gnss_pose.pose.pose, latest_gnss_pose.pose.pose);
    const auto [ekf_diff_unit_vector, ekf_diff_norm] = compute_position_diff_unit_vector(last_reliable_gnss_stamped_ekf_pose, latest_gnss_stamped_ekf_pose);

    // if the difference of ekf is too small, angle check is not reliable
    if(ekf_diff_norm < outlier_detection_distance_threshold_) {
      return false;
    }

    // RCLCPP_WARN(get_logger(), "gnss pose last: (%f, %f), latest: (%f, %f), ekf pose last: (%f, %f), latest: (%f, %f)", last_reliable_gnss_pose.pose.pose.position.x, last_reliable_gnss_pose.pose.pose.position.y, latest_gnss_pose.pose.pose.position.x, latest_gnss_pose.pose.pose.position.y, last_reliable_gnss_stamped_ekf_pose.position.x, last_reliable_gnss_stamped_ekf_pose.position.y, latest_gnss_stamped_ekf_pose.position.x, latest_gnss_stamped_ekf_pose.position.y);

    // RCLCPP_WARN(get_logger(), "gnss vector: (%f, %f), ekf vector: (%f, %f)", gnss_diff_unit_vector.x(), gnss_diff_unit_vector.y(), ekf_diff_unit_vector.x(), ekf_diff_unit_vector.y());

    // compute angle between two vectors
    const double angle_between_gnss_and_ekf = std::acos(gnss_diff_unit_vector.dot(ekf_diff_unit_vector));

    Float64 angle_msg;
    angle_msg.data = angle_between_gnss_and_ekf;
    pub_angle_between_gnss_and_ekf_->publish(angle_msg);

    RCLCPP_DEBUG(get_logger(), "angle: %f", angle_between_gnss_and_ekf);

    if(angle_between_gnss_and_ekf > outlier_detection_angle_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Outlier detected! angle_between_gnss_and_ekf: %f", angle_between_gnss_and_ekf);
      return true;
    }

    return false;
  }

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Publisher<Float64>::SharedPtr pub_angle_between_gnss_and_ekf_;
  rclcpp::Publisher<Int64>::SharedPtr pub_duplication_count_;
  rclcpp::Publisher<Float64>::SharedPtr pub_elapsed_from_last_unique_gnss_received_;
  rclcpp::Publisher<Bool>::SharedPtr pub_duplication_alert_, pub_outlier_alert_, pub_gnss_large_cov_alert_, pub_ekf_large_cov_alert_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_ekf_odom_;

  std::deque<PoseWithCovarianceStamped> gnss_queue_;
  std::deque<Odometry> ekf_odom_queue_;

  // parameters
  size_t gnss_queue_size_;
  size_t ekf_queue_size_;
  double default_gnss_pose_covariance_;
  double gnss_pose_covariance_elapsed_scaling_factor_;
  double standard_gnss_publish_period_;
  double gnss_too_large_cov_threshold_;
  double ekf_too_large_cov_threshold_;
  double outlier_detection_angle_threshold_;
  double outlier_detection_distance_threshold_;
  bool enable_duplicate_detection_;
  bool enable_outlier_detection_;

  // runtime states
  std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> last_reliable_gnss_pose_;
  size_t duplication_count_ = 0;
  size_t unreliable_gnss_count_ = 0;
};

} // namespace aic_tools

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic_tools::GnssFilter>());
  rclcpp::shutdown();
  return 0;
}