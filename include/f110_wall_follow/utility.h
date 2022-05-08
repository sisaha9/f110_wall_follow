//
// Created by yash on 9/24/19.
//

#ifndef SRC_UTILITY_H
#define SRC_UTILITY_H

#include "sensor_msgs/msg/laser_scan.hpp"

namespace wf
{

/// Function to Apply 1d Smoothing Filter (Averaging Filter)
std::vector<double> apply_smoothing_filter(const std::vector<double>& input_vector, size_t smoothing_filter_size)
{
    std::vector<double> smoothened_vector;
    for(size_t i=smoothing_filter_size; i<input_vector.size()-smoothing_filter_size; ++i)
    {
        double current_sum = 0;
        for(size_t j = i-smoothing_filter_size + 1; j<i+smoothing_filter_size; ++j)
        {
            if (j >= 0 && j < input_vector.size()) {
                current_sum += input_vector[j];
            }
        }
        smoothened_vector.push_back(current_sum/(2.0*smoothing_filter_size - 1.0));
    }
    return smoothened_vector;
}

/// Returns Indices of Start and End of the Lidar Scan for the input truncation angle converage
std::vector<double > truncate(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg,
                                                          const double truncation_angle_coverage)
{
    const auto truncated_range_size = static_cast<size_t >(
            (truncation_angle_coverage/(scan_msg->angle_max - scan_msg->angle_min))*scan_msg->ranges.size());
    const size_t start_index = (scan_msg->ranges.size()/2) - (truncated_range_size/2);
    const size_t end_index = (scan_msg->ranges.size()/2) + (truncated_range_size/2);
    return std::vector<double >(scan_msg->ranges.begin()+start_index, scan_msg->ranges.begin()+end_index);
}

} // namespace wf

#endif //SRC_UTILITY_H
