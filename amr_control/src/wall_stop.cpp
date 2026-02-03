#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class WallStop : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

public:
    WallStop() : Node("wall_stop_node") {
        // Subscribe to the LIDAR data
        // "Best Effort" QoS is often needed for Gazebo sensors
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&WallStop::scan_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Wall Stop Node Started!");
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 1. Find the closest obstacle
        // The 'ranges' array contains 360 distances (one for each degree)
        float min_distance = 100.0;
        
        for (float range : msg->ranges) {
            // Ignore invalid readings (infinity or 0)
            if (range < msg->range_max && range > msg->range_min) {
                if (range < min_distance) {
                    min_distance = range;
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Closest Obstacle: %.2f m", min_distance);

        // 2. Logic: Stop if too close
        auto cmd = geometry_msgs::msg::Twist();
        
        if (min_distance < 0.5) {
            // EMERGENCY STOP
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_WARN(this->get_logger(), "TOO CLOSE! STOPPING!");
        } else {
            // Safe to drive forward slowly
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallStop>());
    rclcpp::shutdown();
    return 0;
}