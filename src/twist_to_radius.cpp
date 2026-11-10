#include <memory>
#include <functional>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class TwistToRadiusNode : public rclcpp::Node
{
public:
    TwistToRadiusNode()
    : Node("array_publisher")
    {
        array_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("radius_vel", 10);
        sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_raw", 10,
            std::bind(&TwistToRadiusNode::callback, this, std::placeholders::_1));
    }

private:
    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        std_msgs::msg::Float32MultiArray array;
        double turning_radius;
        const double linear_velocity = msg->linear.x;
        if (msg->angular.z != 0.0) {
            turning_radius = msg->linear.x / msg->angular.z;
        } else {
            turning_radius = 0.0;
        }
        array.data.push_back(turning_radius);
        array.data.push_back(linear_velocity);
        array_pub_->publish(array);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr array_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistToRadiusNode>());
    rclcpp::shutdown();
    return 0;
}