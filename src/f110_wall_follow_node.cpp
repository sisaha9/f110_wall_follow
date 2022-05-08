#include "f110_wall_follow/utility.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node
{
public:
    WallFollow(const rclcpp::NodeOptions & options):  rclcpp::Node("wall_follow_node", options)
    {
        this->declare_parameter<double>("kp");
        kp_ = this->get_parameter("kp").as_double();
        this->declare_parameter<double>("ki");
        ki_ = this->get_parameter("ki").as_double();
        this->declare_parameter<double>("kd");
        kd_ = this->get_parameter("kd").as_double();
        prev_error_ = 0.0;
        error_ = 0.0;
        integral_ = 0.0;
        this->declare_parameter<double>("desired_distance_left");
        desired_left_wall_distance_ = this->get_parameter("desired_distance_left").as_double();
        this->declare_parameter<double>("lookahead_distance");
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        prev_reading_time_ = now().seconds();
        current_reading_time = now().seconds();
        this->declare_parameter<double>("low_error_velocity");
        error_based_velocities_["low"] = this->get_parameter("low_error_velocity").as_double();
        this->declare_parameter<double>("medium_error_velocity");
        this->declare_parameter<double>("medium_error_threshold");
        error_based_velocities_["medium"] = this->get_parameter("medium_error_velocity").as_double();
        error_based_thresholds_["medium"] = this->get_parameter("medium_error_threshold").as_double();
        this->declare_parameter<double>("high_error_velocity");
        this->declare_parameter<double>("high_error_threshold");
        error_based_velocities_["high"] = this->get_parameter("high_error_velocity").as_double();
        error_based_thresholds_["high"] = this->get_parameter("high_error_threshold").as_double();
        this->declare_parameter<double>("truncated_coverage_angle");
        truncated_coverage_angle_ = this->get_parameter("truncated_coverage_angle").as_double();
        this->declare_parameter<int>("smoothing_filter_size");
        smoothing_filter_size_ = this->get_parameter("smoothing_filter_size").as_int();

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "nav",
            rclcpp::SensorDataQoS()
        );
    }

    std::vector<double> preprocess_scan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &scan_msg)
    {
        auto truncated_ranges = wf::truncate(scan_msg, truncated_coverage_angle_);
        for(auto& range : truncated_ranges)
        {
            if(std::isnan(range))
            {
                range = 0;
            }
        }
        return wf::apply_smoothing_filter(truncated_ranges, smoothing_filter_size_);
    }

    /// Returns the distance from obstacle at a given angle from the Laser Scan Message
    double get_range_at_angle(const std::vector<double> &filtered_scan, const double& angle,
            const double angle_increment) const
    {
        const double corrected_angle = angle + (truncated_coverage_angle_/2);
        RCLCPP_INFO(this->get_logger(), "Corrected Angle : %f", corrected_angle);

        const double required_range_index = static_cast<int>(floor(corrected_angle/angle_increment));
        RCLCPP_INFO(this->get_logger(), "Required Range Index : %f", required_range_index);

        RCLCPP_INFO(this->get_logger(), "Required Range Value : %f", filtered_scan[required_range_index]);
        return filtered_scan[required_range_index];
    }

    /// PID controller to control the steering of the car and adjust the velocity accordingly
    void control_steering()
    {
        prev_reading_time_ = current_reading_time;
        current_reading_time = now().seconds();
        const auto dt = current_reading_time - prev_reading_time_;

        integral_ += error_;

        double steering_angle = kp_ * error_ + kd_ * (error_ - prev_error_) / dt + ki_ * (integral_);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = now();
        drive_msg.header.frame_id = "laser";

        if(std::isnan(steering_angle))
        {
            drive_msg.drive.speed = 0;
            drive_msg.drive.steering_angle = 0;
        }

        // Thresholding for limiting the movement of car wheels to avoid servo locking
        if(steering_angle > 0.4)
        {
            steering_angle = 0.4;
        }
        else if(steering_angle < -0.4)
        {
            steering_angle = -0.4;
        }

        RCLCPP_INFO(this->get_logger(), "Steering Angle : %f", steering_angle);
        drive_msg.drive.steering_angle = steering_angle;

        if(abs(steering_angle) > error_based_thresholds_["high"])
        {
            drive_msg.drive.speed = error_based_velocities_["high"];
        }
        else if(abs(steering_angle) > error_based_thresholds_["medium"])
        {
            drive_msg.drive.speed = error_based_velocities_["medium"];
        }
        else
        {
            drive_msg.drive.speed = error_based_velocities_["low"];
        }
        drive_pub_->publish(drive_msg);

        prev_error_ = error_;
    }

    /// Returns value of Error between the required distance and the current distance
    void get_error(const std::vector<double> &filtered_ranges, const double angle_increment)
    {
        const auto distance_of_a = get_range_at_angle(filtered_ranges, 0.5, angle_increment);
        const auto distance_of_b = get_range_at_angle(filtered_ranges, 1.4, angle_increment);
        constexpr auto theta = 0.9;

        const auto alpha = std::atan2(distance_of_a*cos(theta)-distance_of_b,distance_of_a*sin(theta));
        RCLCPP_INFO(this->get_logger(), "alpha: %f", alpha);

        const auto distance_t = distance_of_b*cos(alpha);
        const auto distance_tplus1 = distance_t + lookahead_distance_*sin(alpha);

        error_ = distance_tplus1 - desired_left_wall_distance_ ;
    }

    /// Scan Callback Function
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        const auto filtered_ranges = preprocess_scan(scan_msg);
        get_error(filtered_ranges, scan_msg->angle_increment);
        control_steering();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double kp_, ki_, kd_;
    double prev_error_, error_;
    double integral_;

    double prev_reading_time_;
    double current_reading_time;

    double desired_left_wall_distance_;
    double lookahead_distance_;
    double truncated_coverage_angle_;

    int smoothing_filter_size_;
    std::map<std::string, double> error_based_velocities_;
    std::map<std::string, double> error_based_thresholds_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options{};
    auto node = std::make_shared<WallFollow>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
