#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class Patrol : public rclcpp::Node{
public:
    Patrol()
    : Node("node_name")
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, 
            &Patrol::scan_callback, this, std::placeholders::_1);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
    {
        scan_ = msg;

        // Using only the front 180 degrees: [-pi/2, pi/2]
        double front_left = M_PI/2.0f;
        double front_right = -M_PI/2.0f;

        int start_index = (front_right - scan_.angle_min) / scan_.angle_increment;
        int end_index = (front_left - scan_.angle_min) / scan_.angle_increment;

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    sensor_msgs::msg::LaserScan scan_;
};