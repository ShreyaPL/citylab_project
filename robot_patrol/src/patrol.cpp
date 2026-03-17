#include "rclcpp/utilities.hpp"
#include <chrono>
#include <algorithm>
#include <functional>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class Patrol : public rclcpp::Node{
public:
    Patrol()
    : Node("robot_patrol_node"), 
     linear_(0.1), 
     direction_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10, 
            std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
        
        //publisher object
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/fastbot_1/cmd_vel", 10);

        // timer object
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),     //10 Hz
            std::bind(&Patrol::control_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Robot patrol node started.");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /fastbot_1/scan");
        RCLCPP_INFO(this->get_logger(), "Publishing velocity commands to /fastbot_1/cmd_vel");
        RCLCPP_INFO(this->get_logger(), "Control loop frequency: 10 Hz");
    }

private:
    int angle_to_index(double angle) const
    {
        int idx = static_cast<int>((angle - scan_.angle_min) / scan_.angle_increment);
        return idx;
    }

    double index_to_angle(int index) const
    {
        double angle = scan_.angle_min + index * scan_.angle_increment;
        return angle;
    }

    double get_sector_min_distance(double center_angle, double half_width_rad)
    {
        int center_index = angle_to_index(center_angle);
        int half_window = static_cast<int>(half_width_rad/ scan_.angle_increment);

        double min_distance = scan_.range_max;
        bool found_valid = false;

        for(int i = center_index - half_window; i <= center_index + half_window; i++)
        {
            if(i < 0 || i >= static_cast<int>(scan_.ranges.size())){
                continue;
            }
            double r = scan_.ranges[i];
            if(std::isinf(r)){
                r = scan_.range_max;
            }
            if(std::isnan(r)){
                continue;
            }
            if(r < scan_.range_min || r > scan_.range_max){
                continue;
            }

            found_valid = true;
            min_distance = std::min(min_distance, r);
        }

        if(!found_valid){
            return -1.0f;
        }
        return min_distance;
    }

    double score_direction(int candidate_index, int neighborhood_half_window)
    {
        // This is to avoid narrow gaps
        double min_distance = std::numeric_limits<double>::infinity();
        bool found_valid = false;

        for(int j = candidate_index - neighborhood_half_window;
            j <= candidate_index + neighborhood_half_window; j++)
        {
            if(j < 0 || j >= static_cast<int>(scan_.ranges.size())){
                continue;
            }

            double r = scan_.ranges[j];
            if(std::isinf(r)){
                r = scan_.range_max;
            }
            if(std::isnan(r)){
                continue;
            }
            if(r < scan_.range_min || r > scan_.range_max){
                continue;
            }

            found_valid = true;
            min_distance = std::min(min_distance, r);
        }

        if(!found_valid){
            return -1.0f;
        }

        return min_distance;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_ = *msg;

        if(scan_.ranges.empty()){
            RCLCPP_WARN(this->get_logger(), "Received LaserScan with empty ranges.");
            return;
        }

        // Using only the front 180 degrees: [-pi/2, pi/2]
        const double front_left = M_PI/2.0f;
        const double front_right = -M_PI/2.0f;
        const double front = 0.0;

        int start_index = angle_to_index(front_right);
        int end_index = angle_to_index(front_left);      

        // Use front sector
        double distance_front = get_sector_min_distance(front, 10.0 * M_PI/180); //sector of +/- 10 deg
        // RCLCPP_DEBUG(this->get_logger(),
        //              "Front sector distance: %.2f m, current direction: %.2f rad",
        //              distance_front, direction_);

        if (distance_front < 0.0) {
            // No reliable front data.
            RCLCPP_WARN(this->get_logger(), "No reliable front-sector laser data available.");
            direction_ = 0.0;
            return;
        }

        if (distance_front > 0.35)
        {
            // move 
            RCLCPP_INFO(this->get_logger(),
                        "Path clear. front_distance=%.2f m. Moving straight.",
                        distance_front);
            direction_ = 0.0;
            return;
        }
        else
        {
            // obstacle ahead - search for safest direction
            double best_score = -1.0;
            int best_index = angle_to_index(0.0);

            int neighborhood_half_window =
            std::max(1, static_cast<int>((8.0 * M_PI / 180.0) / scan_.angle_increment));

            for(int i = start_index; i <= end_index; i++)
            {
                double score = score_direction(i, neighborhood_half_window);

                if (score < 0.0)
                {
                    continue;
                }

                double angle = index_to_angle(i);
                
                // Small directional bias
                double biased_score = score;
                if(prefer_clockwise_){
                    if(angle < 0.0){
                        biased_score += 0.05;
                    }
                }
                else{
                    if(angle > 0.0){
                        biased_score += 0.05;
                    }
                }
                if (biased_score > best_score) {
                    best_score = biased_score;
                    best_index = i;
                }
            }

            if(best_score < 0.0){
                RCLCPP_WARN(this->get_logger(),
                        "No valid safe direction found. Defaulting to straight.");
                direction_ = 0.0;
            }else {
                direction_ = index_to_angle(best_index);
            }
            RCLCPP_INFO(this->get_logger(),"Best_score=%.3f  best_angle=%.3f",best_score,direction_);
        }
    }

    void control_callback()
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = linear_;
        msg.angular.z = direction_ / 2.0;
        publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

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