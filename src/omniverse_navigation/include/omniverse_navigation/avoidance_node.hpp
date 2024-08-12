#ifndef AVOIDANCE_NODE_HPP
#define AVOIDANCE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class AvoidanceNode : public rclcpp::Node {
public:
    AvoidanceNode();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();
    void calculate_forces();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_vel_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;

    double repulsive_strength_;
    double repulsive_distance_;
    double speed_;
    int frequency_;
    double max_force_;
};

#endif // AVOIDANCE_NODE_HPP
