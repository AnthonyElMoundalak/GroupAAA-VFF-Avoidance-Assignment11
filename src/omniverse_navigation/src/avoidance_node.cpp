#include "../include/omniverse_navigation/avoidance_node.hpp"
#include <vector>
#include <cmath>

AvoidanceNode::AvoidanceNode() : Node("avoidance_node") {
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&AvoidanceNode::laser_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    rviz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("debugging", qos_profile);

    // repulsive_strength_ = 0.05;
    // repulsive_distance_ = 0.55;
    // speed_ = 0.4;
    // frequency_ = 20;

    this->declare_parameter("repulsive_strength", 0.05);
    repulsive_strength_ = this->get_parameter("repulsive_strength").as_double();

    this->declare_parameter("repulsive_distance", 0.55);
    repulsive_distance_ = this->get_parameter("repulsive_distance").as_double();

    this->declare_parameter("speed", 0.4);
    speed_ = this->get_parameter("speed").as_double();

    this->declare_parameter("frequency", 20);
    frequency_ = this->get_parameter("frequency").as_int();

    this->declare_parameter("max_force", 0.6);
    max_force_ = this->get_parameter("max_force").as_double();


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
    publish_markers();

}


void AvoidanceNode::calculate_forces(){

    float repulsive_force_x = 0.0;
    float repulsive_force_y = 0.0;


    repulsive_force_.resize(2, 0.0);
    attractive_force_.resize(2, 0.0);
    resultant_force_.resize(2, 0.0);

    

    double closest_obstacle_distance = *std::min_element(laser_scan_->ranges.begin(), laser_scan_->ranges.end());
    auto min_element_iter = std::min_element(laser_scan_->ranges.begin(), laser_scan_->ranges.end());
    int i = std::distance(laser_scan_->ranges.begin(), min_element_iter);

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
        if (force > max_force_) force = max_force_;
        repulsive_force_x += -force * std::cos(angle);
        repulsive_force_y += -force * std::sin(angle);
        repulsive_force_.clear();
        repulsive_force_.push_back(repulsive_force_x);
        repulsive_force_.push_back(repulsive_force_y);

    }

    float attractive_force_x = 0.5;
    float attractive_force_y = 0.0;
    attractive_force_.clear();
    attractive_force_.push_back(attractive_force_x);
    attractive_force_.push_back(attractive_force_y);

    float resultant_force_x = attractive_force_x + repulsive_force_x;
    float resultant_force_y = attractive_force_y + repulsive_force_y;
    
    resultant_force_.clear();
    resultant_force_.push_back(resultant_force_x);
    resultant_force_.push_back(resultant_force_y);

    cmd_vel_msg_.linear.x = resultant_force_x;
    cmd_vel_msg_.angular.z = std::atan2(resultant_force_y, resultant_force_x);

}


void AvoidanceNode::publish_markers(){

     // Delete old markers
    for (auto& marker : rviz_msg_.markers) {
        marker.action = visualization_msgs::msg::Marker::DELETE;
    }
    rviz_pub_->publish(rviz_msg_);

    // Clear the marker array for new markers
    rviz_msg_.markers.clear();
    int marker_id = 0;

    visualization_msgs::msg::Marker repulsive_marker;
    repulsive_marker.header.frame_id = "base_link";
    repulsive_marker.header.stamp = this->now();
    repulsive_marker.ns = "repulsive_force";
    repulsive_marker.id = marker_id++;
    repulsive_marker.type = visualization_msgs::msg::Marker::ARROW;
    repulsive_marker.action = visualization_msgs::msg::Marker::ADD;
    repulsive_marker.pose.orientation.w = 1.0;
    repulsive_marker.scale.x = 0.02;
    repulsive_marker.scale.y = 0.05;
    repulsive_marker.scale.z = 0.05;
    repulsive_marker.color.a = 1.0; 
    repulsive_marker.color.r = 1.0;
    repulsive_marker.color.g = 0.0;
    repulsive_marker.color.b = 0.0;

    geometry_msgs::msg::Point start;
    start.x = 0;
    start.y = 0;
    start.z = 0;
    

    geometry_msgs::msg::Point end;
   if (repulsive_force_.size() >= 2) {
    end.x = repulsive_force_[0];
    end.y = repulsive_force_[1];
    end.z = 0;
} else {
    // Handle the error or initialize with default values
    end.x = 0.0;
    end.y = 0.0;
    end.z = 0.0;
}
    

    repulsive_marker.points.push_back(start);
    repulsive_marker.points.push_back(end);

    rviz_msg_.markers.push_back(repulsive_marker);

    // Add attractive force to marker array
    visualization_msgs::msg::Marker attractive_marker;
    attractive_marker.header.frame_id = "base_link";
    attractive_marker.header.stamp = this->now();
    attractive_marker.ns = "attractive_force";
    attractive_marker.id = marker_id++;
    attractive_marker.type = visualization_msgs::msg::Marker::ARROW;
    attractive_marker.action = visualization_msgs::msg::Marker::ADD;
    attractive_marker.pose.orientation.w = 1.0;
    attractive_marker.scale.x = 0.02;
    attractive_marker.scale.y = 0.05;
    attractive_marker.scale.z = 0.05;
    attractive_marker.color.a = 1.0;
    attractive_marker.color.r = 0.0;
    attractive_marker.color.g = 0.0;
    attractive_marker.color.b = 1.0;

    geometry_msgs::msg::Point attractive_start;
    attractive_start.x = 0;
    attractive_start.y = 0;
    attractive_start.z = 0;
   

    geometry_msgs::msg::Point attractive_end;
   if (attractive_force_.size() >= 2) {
    attractive_end.x = attractive_force_[0];
    attractive_end.y = attractive_force_[1];
    attractive_end.z = 0.0;
} else {
    // Handle the error or initialize with default values
    attractive_end.x = 0.0;
    attractive_end.y = 0.0;
    attractive_end.z = 0.0;
}
    

    attractive_marker.points.push_back(attractive_start);
    attractive_marker.points.push_back(attractive_end);

    rviz_msg_.markers.push_back(attractive_marker);

    // Add resultant force to marker array
    visualization_msgs::msg::Marker resultant_marker;
    resultant_marker.header.frame_id = "base_link";
    resultant_marker.header.stamp = this->now();
    resultant_marker.ns = "resultant_force";
    resultant_marker.id = marker_id++;
    resultant_marker.type = visualization_msgs::msg::Marker::ARROW;
    resultant_marker.action = visualization_msgs::msg::Marker::ADD;
    resultant_marker.pose.orientation.w = 1.0;
    resultant_marker.scale.x = 0.02;
    resultant_marker.scale.y = 0.05;
    resultant_marker.scale.z = 0.05;
    resultant_marker.color.a = 1.0;
    resultant_marker.color.r = 0.0;
    resultant_marker.color.g = 1.0;
    resultant_marker.color.b = 0.0;

    geometry_msgs::msg::Point resultant_start;
    resultant_start.x = 0;
    resultant_start.y = 0;
    resultant_start.x = 0;

    geometry_msgs::msg::Point resultant_end;
   if (resultant_force_.size() >= 2) {
    resultant_end.x = resultant_force_[0];
    resultant_end.y = resultant_force_[1];
    resultant_end.z = 0.0;
} else {
    // Handle the error or initialize with default values
    resultant_end.x = 0.0;
    resultant_end.y = 0.0;
}

    resultant_marker.points.push_back(resultant_start);
    resultant_marker.points.push_back(resultant_end);

    rviz_msg_.markers.push_back(resultant_marker);

    rviz_pub_->publish(rviz_msg_);

}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AvoidanceNode>());
    rclcpp::shutdown();
    return 0;
}
