#ifndef VFF_AVOIDANCE_HPP
#define VFF_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Vector3.h>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>




class VFFAvoidance : public rclcpp::Node {
public:
    VFFAvoidance();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    tf2::Vector3 calculateRepulsiveVector(const sensor_msgs::msg::LaserScan::SharedPtr& msg);
    double calculateAngularVelocity(double angle);
    double clamp(double value, double min, double max);
    
    

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

   

    
    tf2::Vector3 fixed_global_direction_;
};

#endif // VFF_AVOIDANCE_HPP
