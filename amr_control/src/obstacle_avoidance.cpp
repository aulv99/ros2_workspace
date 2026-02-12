#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" 
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm> 
#include <fstream>
#include <cstdlib> // for std::getenv

using std::placeholders::_1;

class ObstacleAvoidance : public rclcpp::Node {
private:
    // sub and pub topics: odometry, lidar scan, commands
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // for file output
    std::ofstream data_file_;

    // target
    double target_x_ = 2;
    double target_y_ = 0;

    // robot state
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    // sensor state
    double left_dist_ = 999.0;
    double center_dist_ = 999.0;
    double right_dist_ = 999.0;

    // state change timer
    rclcpp::Time last_state_change_;
    int current_state_ = 0;

public:
    ObstacleAvoidance() : Node("obstacle_avoidance_node") {
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        // subscribe to position
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&ObstacleAvoidance::odom_callback, this, _1));

        // subscribe to vision
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&ObstacleAvoidance::scan_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Active. Target (%.1f, %.1f)", target_x_, target_y_);

        last_state_change_ = this->get_clock()->now();

        std::string path = std::string(std::getenv("HOME")) + "/trajectory.txt";
        data_file_.open(path, std::ios::out | std::ios::trunc);
        if (data_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "Logging trajectory to: %s", path.c_str());
        }
    }

    ~ObstacleAvoidance() {
        // close data file saving
        if (data_file_.is_open()) {
            data_file_.close();
            printf("Data file saved successfully.\n");
        }
    }

    // sensor processing
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // lidar gives 360 values. simplifying it to 3 sectors
        // index 0 is front, 90 is left, 270 is right

        // helper to handle infinity values
        auto get_range = [&](int index) {
            double r = msg->ranges[index];
            return std::isinf(r) ? 3.5 : r;
        };

        // scan front sector
        // check a cone in front
        double min_front = 999.0;
        for (int i = 0; i < 30; i++) min_front = std::min(min_front, get_range(i));
        for (int i = 330; i < 360; i++) min_front = std::min(min_front, get_range(i));
        center_dist_ = min_front;

        // scan left sector
        double min_right = 999.0;
        for (int i = 200; i < 320; i++) min_right = std::min(min_right, get_range(i));
        right_dist_ = min_right;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

        // update position
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        // saving position information to a data file for plotting purposes
        if (data_file_.is_open()) {
            data_file_ << x_ << " " << y_ << "\n";
        }

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);

        // calculate goal logic
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double dist_error = std::sqrt(dx*dx + dy*dy);
        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = desired_yaw - yaw_;
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        auto cmd = geometry_msgs::msg::Twist();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
            "F: %.2f | R: %.2f | Rule: ?", center_dist_, right_dist_);

        // logic switching 
        // state based approach to switch between linear motion and obstacle avoidance
        int proposed_state = 0;
        if (center_dist_ < 0.33) proposed_state = 1;
        else if (right_dist_ < 0.33) proposed_state = 2;
        else proposed_state = 0;
        
        auto now = this->get_clock()->now();
        // use timer to avoid switching state too rapidly when sensor data is noisy
        if ((now - last_state_change_).seconds() > 0.4 || proposed_state == 1) {
            if (current_state_ != proposed_state) {
                current_state_ = proposed_state;
                last_state_change_ = now;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
        }
        // obstacle in front of the robot
        if (current_state_ == 1) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3;
        }
        // obstacle in the right side of the robot
        else if (current_state_ == 2) {
            cmd.linear.x = 0.1;
            double error = 0.35 - right_dist_;
            cmd.angular.z = std::max(std::min(error * 1.0, 0.4), -0.4);
        }
        else {
            if (dist_error < 0.2) {
                // Arrived at the target
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Target Reached!");
            } else {
                // standard P-Controller to target
                // Only allow forward motion if we are roughly facing the target, otherwise rotate
                if (std::abs(yaw_error) > 0.3) {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.5 * yaw_error;
                } else {
                    cmd.linear.x = 0.1;
                    cmd.angular.z = 0.5 * yaw_error;
                }
            }
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());
    rclcpp::shutdown();
    return 0;
}