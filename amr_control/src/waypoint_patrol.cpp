#include "rclcpp/rclcpp.hpp" // ROS library
#include "nav_msgs/msg/odometry.hpp" // 
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class WaypointPatrol : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::vector<std::pair<double, double>> waypoints_;
    size_t current_wp_index_ = 0;
    bool loop_forever_ = true;

    // current state
    double x_ = 0.0;    
    double y_ = 0.0;
    double yaw_ = 0.0;

    // bool goal_reached_ = false;

public:
    WaypointPatrol() : Node("waypoint_patrol_node") {
        // QoS for odom
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&WaypointPatrol::odom_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        waypoints_ = {
            {-0.5, -0.5}, // 1. Top Right
            {-0.5,  0.5}, // 2. Top left
            { 0.5,  0.5}, // 3. Bottom left
            { 0.5, -0.5},  // 4. Bottom right (start)
            { 1.5, -0.5},
            { 1.5,  0.5},
            {-2.0,  0.5},
            {-2.0, -0.5},
        };

        RCLCPP_INFO(this->get_logger(), "Patrol Started with %ld waypoints!", waypoints_.size());

    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // get position
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        // get orientation (quaternion -> yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
        
        if (current_wp_index_ >= waypoints_.size()) {
            if (loop_forever_) {
                current_wp_index_ = 0; // Restart path
                RCLCPP_INFO(this->get_logger(), "Looping back to start!");
            } else {
                stop_robot();
                return;
            }
        }

        auto target = waypoints_[current_wp_index_];
        double dx = target.first - x_;
        double dy = target.second - y_;
        double distance_error = std::sqrt(dx*dx + dy*dy);

        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = desired_yaw - yaw_;

        // angle normalisation (error between -pi and +pi)
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        // 3. Check Index Logic
        if (distance_error < 0.10) { // 10cm tolerance
            RCLCPP_INFO(this->get_logger(), "Reached WP #%ld: (%.1f, %.1f)", 
                        current_wp_index_, target.first, target.second);
            current_wp_index_++;
            return; // Wait for next callback to calculate new target
        }

        // 4. Move (Pivot Turn Logic)
        geometry_msgs::msg::Twist cmd;
        
        if (std::abs(yaw_error) > 0.3) {
            // Turn in place if not facing target
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5 * yaw_error;
        } else {
            // Drive and steer
            cmd.linear.x = 0.2 * distance_error;
            cmd.angular.z = 1.0 * yaw_error;

            if (cmd.linear.x > 0.20 ) cmd.linear.x = 0.20;
        }
        
        cmd_pub_->publish(cmd);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char * argv []) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointPatrol>());
    rclcpp::shutdown();
    return 0;
}
