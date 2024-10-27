#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include <deque>

namespace aic_tools {

class GnssFilter : public rclcpp::Node {
public:
  using PoseWithCovarianceStamped =
      geometry_msgs::msg::PoseWithCovarianceStamped;
  using Odometry = nav_msgs::msg::Odometry;

  GnssFilter() : Node("gnss_filter") {
    // load parameters
    gnss_queue_size_ =
        static_cast<size_t>(declare_parameter<int64_t>("gnss_queue_size"));
    outlier_threshold_ = declare_parameter<double>("outlier_threshold");

    // create pub/sub
    pub_gnss_ = this->create_publisher<PoseWithCovarianceStamped>(
        "/sensing/gnss/pose_with_covariance_filtered",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

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
    push_to_queue(*msg);

    // drop outlier message
    if (is_outlier(*msg)) {
      RCLCPP_WARN(get_logger(), "Dropped outlier message");
      return;
    }

    // publish the filtered message
    pub_gnss_->publish(*msg);
    RCLCPP_DEBUG(get_logger(), "Published filtered message");
  }

  void ekf_odom_callback(const Odometry::SharedPtr msg) {}

  void push_to_queue(const PoseWithCovarianceStamped &msg) {
    if (gnss_queue_.size() >= gnss_queue_size_) {
      gnss_queue_.pop_front();
    }
    gnss_queue_.push_back(msg);
  }

  bool is_duplicate(const PoseWithCovarianceStamped &p0,
                    const PoseWithCovarianceStamped &p1) {
    return p0.pose.pose.position.x == p1.pose.pose.position.x &&
           p0.pose.pose.position.y == p1.pose.pose.position.y &&
           p0.pose.pose.position.z == p1.pose.pose.position.z;
  }

  bool is_duplicate(const PoseWithCovarianceStamped &msg) {
    for (const auto &p : gnss_queue_) {
      if (is_duplicate(p, msg)) {
        return true;
      }
    }
    return false;
  }

  bool is_outlier(const PoseWithCovarianceStamped &msg) { return false; }

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_gnss_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_ekf_odom_;

  std::deque<PoseWithCovarianceStamped> gnss_queue_;

  // parameters
  size_t gnss_queue_size_;
  double outlier_threshold_;
};

} // namespace aic_tools

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aic_tools::GnssFilter>());
  rclcpp::shutdown();
  return 0;
}
