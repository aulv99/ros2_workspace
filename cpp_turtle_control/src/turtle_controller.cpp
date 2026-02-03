#include "rclcpp/rclcpp.hpp" // ROS Client Library for C++
#include "geometry_msgs/msg/twist.hpp" // geometry_msgs provides messages for common geometric primitives such as points, vectors, and poses
#include "turtlesim/msg/pose.hpp" // Spawns a turtle at (x, y, theta) and returns the name of the turtle

using std::placeholders::_1; // a namespace alias 

class TurtleController : public rclcpp::Node {
// Inheriting ROS 2 node class
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // Publishing twist to control motion
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_; // Subscribing pose to acquire location information

    // Targets
    double target_x_ = 9.0; // Target x position
    double target_y_ = 9.0; // Target y position

    // tuning parameters
    double kp_linear_ = 1.0; // linear gain parameter
    double kp_angular_ = 4.0; // angular gain parameter

public:
    TurtleController() : Node("turtle_controller") {
        this->declare_parameter("kp_linear", 1.0);
        this->declare_parameter("kp_angular", 4.0);
        this->declare_parameter("target_x", 9.0);
        this->declare_parameter("target_y", 9.0);

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10); // Publishing to topic "/turtle1/cmd_vel"
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&TurtleController::pose_callback, this, _1)); // Subscribing to topic "/turtle1/pose"
            
        RCLCPP_INFO(this->get_logger(), "Controller Started. Target X=%.2f", target_x_); // RCLCPP_INFO prints to the terminal AND sends the log to the ROS system, adds a timestamp, and colors it green/white automatically.
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        // Fetching current values for live tuning
        kp_linear_ = this->get_parameter("kp_linear").as_double(); 
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        
        // Calculating errors
        double dx = target_x_ - msg->x;
        double dy = target_y_ - msg->y;

        double distance_error = std::sqrt(dx*dx + dy*dy); // Distance formula (pythgoras)
        double desired_theta = std::atan2(dy, dx);
        double angle_error = desired_theta - msg->theta;

        auto cmd = geometry_msgs::msg::Twist(); // Preparing twist message to send to the motors
        
        if (distance_error > 0.1) {
            cmd.linear.x = kp_linear_ * distance_error; // Linear P-controller
            cmd.angular.z = kp_angular_ * angle_error;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        publisher_->publish(cmd); // Publishing to "/turtle1/cmd_vel"
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
    return 0;
}