#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Patrol : public rclcpp::Node{
public:
    Patrol()
    : Node("node_name")
    {
        laserscan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, 
            &Patrol::laserscan_callback, this, std::placeholders::_1);
    }

private:
    void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr & msg)
    {}

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_subscriber_;
};