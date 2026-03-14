#include "rclcpp/utilities.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Patrol : public rclcpp::Node{
public:
    Patrol()
    : Node("node_name"), linear_(0.1), direction_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, 
            std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        //publisher object
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("fastbot_1/cmd_vel", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = *msg;

        // Using only the front 180 degrees: [-pi/2, pi/2]
        double front_left = M_PI/2.0f;
        double front_right = -M_PI/2.0f;
        double front = 0.0;

        int start_index = static_cast<int>((front_right - scan_.angle_min) / scan_.angle_increment);
        int end_index = static_cast<int>((front_left - scan_.angle_min) / scan_.angle_increment);
        int front_index = static_cast<int>((front - scan_.angle_min) / scan_.angle_increment);
        RCLCPP_INFO(this->get_logger(), "Front index: %d", front_index);

        double distance_front = scan_.ranges[front_index];
        // check if distance_front is valid value
        if(std::isnan(distance_front) || distance_front < scan_.range_min)
        { 
            stop_robot();
            return;
        }
        if(std::isinf(distance_front) || distance_front > scan_.range_max)
        {
            distance_front = scan_.range_max;
        }

        if (distance_front > 0.35)
        {
            // move forward
            direction_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Moving Forward...");

            publish_msg();
        }
        else
        {
            // get largest distance ray
            double max_distance = -1.0;
            int max_index = front_index;
            for(int i = start_index; i <= end_index; i++)
            {
                double ray = scan_.ranges[i];

                // check for valid ray value in scan_.ranges
                if(std::isnan(ray) || std::isinf(ray)){ continue;}
                if(ray < scan_.range_min || ray > scan_.range_max){continue;}

                if(ray > max_distance){
                    max_distance = ray;
                    max_index = i;
                }
            }
            // angle from index
            if(max_distance < 0.0)
            {
                direction_ = 0.0;
            }
            else {
                direction_ = scan_.angle_min + (max_index * scan_.angle_increment);
            }
            RCLCPP_INFO(this->get_logger(), "Direction of maximum distance: %0.2f", direction_);

            publish_msg();
        }
    }

    void publish_msg(){
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_;
        msg.angular.z = direction_ / 2.0f;
        publisher_->publish(msg);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    sensor_msgs::msg::LaserScan scan_;
    
    double linear_;
    double direction_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);

    auto patrol_node = std::make_shared<Patrol>();
    rclcpp::spin(patrol_node);
    rclcpp::shutdown();
    return 0;
}