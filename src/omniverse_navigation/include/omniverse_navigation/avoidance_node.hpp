#ifndef AVOIDANCE_NODE_HPP
#define AVOIDANCE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class AvoidanceNode : public rclcpp::Node {
public:
    AvoidanceNode();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();
    void calculate_forces();
    void publish_markers();

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;
    

    geometry_msgs::msg::Twist cmd_vel_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
    visualization_msgs::msg::MarkerArray rviz_msg_;

    std::vector<float> repulsive_force_;
    std::vector<float> attractive_force_;
    std::vector<float> resultant_force_;

    double repulsive_strength_;
    double repulsive_distance_;
    double speed_;
    int frequency_;
};

#endif // VFF_NODE_HPP
