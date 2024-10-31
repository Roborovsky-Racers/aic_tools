#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/logging.hpp"
#include <angles/angles.h>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <cmath>
#include <deque>

namespace aic_tools {

class GnssFilter : public rclcpp::Node {
public:
  using Float64 = std_msgs::msg::Float64;
  using PoseWithCovarianceStamped =
      geometry_msgs::msg::PoseWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;

  GnssFilter() : Node("gnss_filter") {
    // load parameters
    gnss_queue_size_ =
        static_cast<size_t>(declare_parameter<int64_t>("gnss_queue_size"));
    ekf_keep_duration_ = declare_parameter<double>("ekf_keep_duration");
    gnss_delay_sec_ = declare_parameter<double>("gnss_delay_sec");
    outlier_threshold_ = declare_parameter<double>("outlier_threshold");
    enable_duplicate_detection_ =
        declare_parameter<bool>("enable_duplicate_detection");
    enable_outlier_detection_ =
        declare_parameter<bool>("enable_outlier_detection");

    // create pub/sub
    pub_gnss_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance_filtered",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    pub_distance_ = this->create_publisher<Float64>(
        "~/distance_from_ekf", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_yaw_ = this->create_publisher<Float64>(
        "~/gnss_yaw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
    pub_yaw_diff_ = this->create_publisher<Float64>(
        "~/gnss_yaw_diff", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

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
  void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg) {
    // drop duplicate message
    if (is_duplicate(*msg)) {
      RCLCPP_DEBUG(get_logger(), "Dropped duplicate message");
      return;
    }

    // push unique data to queue
    if (gnss_queue_.size() >= gnss_queue_size_) {
      gnss_queue_.pop_front();
    }
    gnss_queue_.push_back(*msg);

    // drop outlier message
    if (is_outlier(*msg)) {
      RCLCPP_WARN(get_logger(), "Dropped outlier message");
      return;
    }

    // publish the filtered message
    pub_gnss_->publish(*msg);
    RCLCPP_DEBUG(get_logger(), "Published filtered message");
  }

  void ekf_odom_callback(const Odometry::SharedPtr msg) {
    ekf_odom_queue_.push_back(*msg);
  }

  bool is_duplicate(const PoseWithCovarianceStamped &p0,
                    const PoseWithCovarianceStamped &p1) {
    return p0.pose.pose.position.x == p1.pose.pose.position.x &&
           p0.pose.pose.position.y == p1.pose.pose.position.y &&
           p0.pose.pose.position.z == p1.pose.pose.position.z;
  }

  bool is_duplicate(const PoseWithCovarianceStamped &msg) {
    if(enable_duplicate_detection_) {
      return false;
    }

    for (const auto &p : gnss_queue_) {
      if (is_duplicate(p, msg)) {
        return true;
      }
    }
    return false;
  }

  bool is_outlier(const PoseWithCovarianceStamped &msg) {
    if(ekf_odom_queue_.empty()) {
      return false;
    }

    const double yaw = tier4_autoware_utils::getRPY(msg.pose.pose.orientation).z;
    const double yaw_diff = angles::shortest_angular_distance(last_yaw_, yaw);
    const double continuouse_yaw = last_yaw_ + yaw_diff;
    last_yaw_ = yaw;

    const double ekf_yaw = tier4_autoware_utils::getRPY(ekf_odom_queue_.back().pose.pose.orientation).z;
    const double ekf_yaw_diff = angles::shortest_angular_distance(last_ekf_yaw_, ekf_yaw);
    const double continuouse_ekf_yaw = last_ekf_yaw_ + ekf_yaw_diff;
    last_ekf_yaw_ = ekf_yaw;

    Float64 yaw_msg;
    yaw_msg.data = yaw;
    pub_yaw_->publish(yaw_msg);

    // yaw_msg.data = yaw_diff;
    yaw_msg.data = continuouse_yaw - continuouse_ekf_yaw;
    pub_yaw_diff_->publish(yaw_msg);

    if (!enable_outlier_detection_) {
      return false;
    }

    const auto &latest_gnss_time = rclcpp::Time(
        msg.header.stamp.sec, msg.header.stamp.nanosec, RCL_ROS_TIME);

    // drop old ekf odom data
    while (!ekf_odom_queue_.empty() &&
           (latest_gnss_time - ekf_odom_queue_.front().header.stamp).seconds() >
               ekf_keep_duration_) {
      ekf_odom_queue_.pop_front();
    }

    // queueが空の場合は処理しない
    if (ekf_odom_queue_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "ekf_odom_queue is empty");
      return false;
    }

    // gnssの取得遅れを考慮して、ekf_odom の少し過去の値を取得
    const auto target_odom_time =
        latest_gnss_time - rclcpp::Duration::from_seconds(gnss_delay_sec_);

    // 最も近い時刻のekf_odomを取得
    double min_diff_time = std::numeric_limits<double>::max();
    Odometry ekf_odom;
    for (const auto &odom : ekf_odom_queue_) {
      const double diff_time =
          std::abs((target_odom_time - odom.header.stamp).seconds());

      if (diff_time < min_diff_time) {
        min_diff_time = diff_time;
        ekf_odom = odom;
      }
    }

    // gnssとekf_odomの差分を計算
    const double distance =
        std::hypot(msg.pose.pose.position.x - ekf_odom.pose.pose.position.x,
                   msg.pose.pose.position.y - ekf_odom.pose.pose.position.y);

    // publish distance
    Float64 distance_msg;
    distance_msg.data = distance;
    pub_distance_->publish(distance_msg);

    RCLCPP_DEBUG(get_logger(),
                 "distance: %f, min_diff_time: %f, queue_remain: %zu", distance,
                 min_diff_time, ekf_odom_queue_.size());

    // check outlier
    if (distance < outlier_threshold_) {
      return false;
    }

    RCLCPP_WARN(get_logger(),
                "Outlier detected! expected pose (x: %f, y: %f), "
                "actual pose (x: %f, y: %f), distance: %f, min_diff_time: %f",
                ekf_odom.pose.pose.position.x, ekf_odom.pose.pose.position.y,
                msg.pose.pose.position.x, msg.pose.pose.position.y, distance,
                min_diff_time);

    return true;
  }

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Publisher<Float64>::SharedPtr pub_distance_;
  rclcpp::Publisher<Float64>::SharedPtr pub_yaw_, pub_yaw_diff_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_ekf_odom_;

  std::deque<PoseWithCovarianceStamped> gnss_queue_;
  std::deque<Odometry> ekf_odom_queue_;

  // parameters
  size_t gnss_queue_size_;
  double ekf_keep_duration_;
  double outlier_threshold_;
  double gnss_delay_sec_;
  bool enable_duplicate_detection_;
  bool enable_outlier_detection_;

  // runtime states
  double last_yaw_ = 0.0;
  double last_ekf_yaw_ = 0.0;
};

} // namespace aic_tools

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic_tools::GnssFilter>());
  rclcpp::shutdown();
  return 0;
}
