#pragma once

#include "aic_tools/gnss_filter.hpp"
#include <rclcpp/time.hpp>
#include <unordered_map>

namespace aic_tools {

class KartDiag : public GnssFilter{

public:
  KartDiag();

private:
virtual void gnss_callback(const PoseWithCovarianceStamped::SharedPtr msg) override;
void on_shutdown();
void update_duplicate_count_map();
void print_stats();

// Duplication detection
size_t gnss_duplicate_count_;
std::unordered_map<size_t, size_t> gnss_duplicate_count_map_;
rclcpp::Time last_gnss_time_{0, 0};

// Parameters
size_t alert_gnss_duplicate_count_;
double alert_gnss_discontinuation_sec_;

};

} // namespace aic_tools