#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "amr_control/simple_pid.hpp"

using std::placeholders::_1;

class WallFollowerPID : public rclcpp::Node {
public:
    WallFollowerPID() : Node("wall_follower_pid") {
        // parameters with default values
        this->declare_parameter("kp", -0.8);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", -1.0);

        // initialize pid
        pid_controller_ = std::make_unique<SimplePID>(-0.8, 0.0, -1.0, -1.0, 1.0);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&WallFollowerPID::scan_callback, this, _1));
        
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        
        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Wall Follower Node Started - Live Tuning Ready");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        if (dt < 0.001) return; 

        // PID live tuning
        double k_p, k_i, k_d;
        this->get_parameter("kp", k_p);
        this->get_parameter("ki", k_i);
        this->get_parameter("kd", k_d);
        
        // new values to the PID controller
        pid_controller_->set_coefficients(k_p, k_i, k_d);
        // -----------------------------

        // scanning a sector to find a closest point
        float min_left_dist = 100.0;
        for (int i = 70; i < 110; i++) {
            float range = msg->ranges[i];
            if (!std::isinf(range) && !std::isnan(range) && range > 0.1) {
                if (range < min_left_dist) min_left_dist = range;
            }
        }

        // control logic
        double control_output = 0.0;
        
        if (min_left_dist > 2.0) {
            // wall lost
            control_output = 0.0; 
        } else {
            // PID calculation
            control_output = pid_controller_->calculate(0.5, min_left_dist, dt);
            
            // logging distance, output, gain parameters
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Dist: %.2f | Cmd: %.2f | Kp: %.2f Kd: %.2f", 
                min_left_dist, control_output, k_p, k_d);
        }

        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.15; 
        twist_msg.angular.z = control_output; 

        cmd_pub_->publish(twist_msg);
        last_time_ = current_time;
    }

    std::unique_ptr<SimplePID> pid_controller_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowerPID>());
    rclcpp::shutdown();
    return 0;
}