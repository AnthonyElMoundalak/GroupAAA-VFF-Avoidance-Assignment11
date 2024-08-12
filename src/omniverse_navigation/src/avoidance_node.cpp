#include "../include/omniverse_navigation/avoidance_node.hpp"
#include <vector>
#include <cmath>

AvoidanceNode::AvoidanceNode() : Node("avoidance_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AvoidanceNode::laser_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    repulsive_strength_ = 0.05;
    repulsive_distance_ = 0.55;
    speed_ = 0.4;
    frequency_ = 20;

    int period_in_milliseconds = 1000 / frequency_;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_in_milliseconds), std::bind(&AvoidanceNode::timer_callback, this));

    cmd_vel_msg_.linear.x = speed_;
    cmd_vel_msg_.angular.z = 0.0;

}


void AvoidanceNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    laser_scan_ = msg;
}


void AvoidanceNode::timer_callback(){
    if (laser_scan_){
        calculate_forces();
        // RCLCPP_INFO(this->get_logger(), "Calculate forces...");
    }
    else{
        cmd_vel_msg_.linear.x = 0.0;
        cmd_vel_msg_.angular.z = 0.0;
    }

    publisher_->publish(cmd_vel_msg_);

}


void AvoidanceNode::calculate_forces(){

    float repulsive_force_x = 0.0;
    float repulsive_force_y = 0.0;

    // double closest_obstacle_distance = *std::min_element(laser_scan_->ranges.begin(), laser_scan_->ranges.end());
    // auto min_element_iter = std::min_element(laser_scan_->ranges.begin(), laser_scan_->ranges.end());
    // int i = std::distance(laser_scan_->ranges.begin(), min_element_iter);


    // Find the minimum element in the first 90 elements
    auto minFirst90Iter = std::min_element(laser_scan_->ranges.begin(), laser_scan_->ranges.begin() + 90);
    float minFirst90 = *minFirst90Iter;
    int posMinFirst90 = std::distance(laser_scan_->ranges.begin(), minFirst90Iter);

    // Find the minimum element in the last 90 elements
    auto minLast90Iter = std::min_element(laser_scan_->ranges.end() - 90, laser_scan_->ranges.end());
    float minLast90 = *minLast90Iter;
    int posMinLast90 = std::distance(laser_scan_->ranges.begin(), minLast90Iter);

    // Compare the two minimums and find the overall minimum and its position
    double closest_obstacle_distance;
    int i;
    if (minFirst90 < minLast90) {
        closest_obstacle_distance = minFirst90;
        i = posMinFirst90;
    } else {
        closest_obstacle_distance = minLast90;
        i = posMinLast90;
    }



    if (closest_obstacle_distance < repulsive_distance_){
        float angle = laser_scan_->angle_min + i * laser_scan_->angle_increment;
        float force = repulsive_strength_ / (closest_obstacle_distance * closest_obstacle_distance);
        if (force > 0.6) force = 0.6;
        repulsive_force_x += -force * std::cos(angle);
        repulsive_force_y += -force * std::sin(angle);
    }

    float attractive_force_x = 0.5;
    float attractive_force_y = 0.0;

    float resultant_force_x = attractive_force_x + repulsive_force_x;
    float resultant_force_y = attractive_force_y + repulsive_force_y;

    cmd_vel_msg_.linear.x = resultant_force_x;
    cmd_vel_msg_.angular.z = std::atan2(resultant_force_y, resultant_force_x);

}



// void AvoidanceNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
//     float repulsive_force_x = 0.0;
//     float repulsive_force_y = 0.0;

//     for (size_t i = 0; i < msg->ranges.size(); ++i) {
//         float range = msg->ranges[i];
//         if (range < msg->range_max && range > msg->range_min) {
//             float angle = msg->angle_min + i * msg->angle_increment;
//             float force = 0.1 / range; // simple inverse distance weighting
//             repulsive_force_x += force * std::cos(angle);
//             repulsive_force_y += force * std::sin(angle);
//         }
//     }

//     // Normalize the repulsive forces
//     float magnitude = std::sqrt(repulsive_force_x * repulsive_force_x + repulsive_force_y * repulsive_force_y);
//     if (magnitude > 0.0) {
//         repulsive_force_x /= magnitude;
//         repulsive_force_y /= magnitude;
//     }

//     // Compute the desired velocity
//     float linear_x = 0.2 - repulsive_force_x;
//     float angular_z = -repulsive_force_y;

//     auto twist_msg = geometry_msgs::msg::Twist();
//     twist_msg.linear.x = linear_x;
//     twist_msg.angular.z = angular_z;

//     publisher_->publish(twist_msg);
// }



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
