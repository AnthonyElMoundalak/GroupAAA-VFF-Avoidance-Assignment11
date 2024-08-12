#include "../include/omniverse_navigation/avoidance_node.hpp"


VFFAvoidance::VFFAvoidance()
: Node("vff_avoidance_node")
{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&VFFAvoidance::laserCallback, this, std::placeholders::_1));

    // Define the fixed global direction vector (e.g., moving forward in the global frame)
    fixed_global_direction_ = tf2::Vector3(1.0, 0.0, 0.0); // Forward direction in the global frame

    RCLCPP_INFO(this->get_logger(), "VFF Avoidance Node has been started.");
}

void VFFAvoidance::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    tf2::Vector3 repulsive_vector = calculateRepulsiveVector(msg);
    geometry_msgs::msg::Twist cmd_vel;

    // Combine the global fixed direction vector with the repulsive vector
    double repulsive_weight = 0.5; // Adjust this value as needed
    tf2::Vector3 resultant_vector = fixed_global_direction_ + repulsive_weight * repulsive_vector;
    RCLCPP_INFO(this->get_logger(), "Resultant Vector: x = %f, y = %f", 
                resultant_vector.x(), resultant_vector.y());

    // Calculate the angular velocity based on the angle of the resultant vector
    double angle = atan2(resultant_vector.y(), resultant_vector.x());

    // If the resultant vector is close to the forward direction, reduce angular velocity
    if (fabs(angle) < 0.1) { // Threshold value, adjust as needed
        angle = 0.0;
    }

    double angular_velocity = calculateAngularVelocity(angle);

    // Set the linear and angular velocity in the Twist message
    cmd_vel.linear.x = clamp(resultant_vector.length(), 0.0, 0.5); // Clamp to a safe range
    cmd_vel.angular.z = angular_velocity;

    // Publish the cmd_vel message
    cmd_vel_pub_->publish(cmd_vel);
}

tf2::Vector3 VFFAvoidance::calculateRepulsiveVector(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
{
    tf2::Vector3 repulsive_vector(0.0, 0.0, 0.0);
    double min_distance = std::numeric_limits<double>::infinity();

    // Define the angular range in front of the robot (e.g., ±45 degrees)
    double forward_angle_min = -M_PI / 4; // -45 degrees
    double forward_angle_max = M_PI / 4;  // +45 degrees

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double distance = msg->ranges[i];
        double angle = msg->angle_min + i * msg->angle_increment;

        // Only consider obstacles within the specified forward angle range
        if (angle >= forward_angle_min && angle <= forward_angle_max) {
            if (distance < min_distance) {
                min_distance = distance;
                repulsive_vector.setX(-cos(angle) / distance); // Push back
                repulsive_vector.setY(-sin(angle) / distance); // Push away
            }
        }
    }

    //RCLCPP_INFO(this->get_logger(), "Repulsive Vector: [%f, %f]", repulsive_vector.x(), repulsive_vector.y());
    return repulsive_vector;
}

double VFFAvoidance::calculateAngularVelocity(double angle)
{
    // Simple proportional control for angular velocity based on the resultant vector's angle
    double kp = 1.0; // Proportional gain
    return clamp(kp * angle, -1.0, 1.0); // Clamp to a safe angular speed range
}

double VFFAvoidance::clamp(double value, double min, double max)
{
    return std::max(min, std::min(value, max));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFFAvoidance>());
    rclcpp::shutdown();
    return 0;
}
