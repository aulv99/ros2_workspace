#include "rclcpp/rclcpp.hpp" // ROS library
#include "nav_msgs/msg/odometry.hpp" // 
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

class GoToGoal : public rclcpp::Node {
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double target_x_ = -1.0; // target location is 1.5 meters
    double target_y_ = -0.5; // we don't move in y dir for now

    // current state
    double current_x_ = 0.0;    
    double current_y_ = 0.0;
    double current_yaw_ = 0.0;

    bool goal_reached_ = false;

public:
    GoToGoal() : Node("go_to_goal_node") {
        // QoS for odom
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&GoToGoal::odom_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "GoToGoal Node Started. Target: %.2f, %.2f", target_x_, target_y_);
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // get position
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // get orientation (quaternion -> yaw)
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance_error = std::sqrt(dx*dx + dy*dy);

        double desired_angle = std::atan2(dy, dx);
        double angle_error = desired_angle - current_yaw_;

        // angle normalisation (error between -pi and +pi)
        while (angle_error > M_PI) angle_error -= 2 * M_PI;
        while (angle_error < -M_PI) angle_error += 2 * M_PI;

        auto cmd = geometry_msgs::msg::Twist();

        if (goal_reached_) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }

        // debug 
        RCLCPP_INFO(this->get_logger(), "Dist: %.2f, Angle: %.2f", distance_error, angle_error);
        
        if (distance_error > 0.15) {
            cmd.linear.x = 0.2 * distance_error;
            cmd.angular.z = 1.0 * angle_error;

            if (cmd.linear.x > 0.15 ) cmd.linear.x = 0.15;

            if (std::abs(angle_error) > 1.5) {
                cmd.linear.x = 0.0; // Stop linear movement if we need to turn around
            }

        } else {
            // arrived
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO_ONCE(this->get_logger(), "Target Reached!");
            goal_reached_ = true;
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char * argv []) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToGoal>());
    rclcpp::shutdown();
    return 0;
}
