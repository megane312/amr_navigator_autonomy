#include <memory>
#include <functional>
#include <chrono>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class SafetyEvaluatorNode : public rclcpp::Node
{
public:
    SafetyEvaluatorNode()
    : Node("safety_evaluator"), close_points_num_(0), course_points_num_(0)
    {
        declare_parameter<int>("close_lim", 50);
        declare_parameter<int>("course_lim", 10);
        get_parameter("close_lim", close_lim_);
        get_parameter("course_lim", course_lim_);

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cmd_raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_raw", 10,
            std::bind(&SafetyEvaluatorNode::cmdRawCallback, this, std::placeholders::_1));
        close_points_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/close_points_num", 10,
            std::bind(&SafetyEvaluatorNode::closePointsCallback, this, std::placeholders::_1));
        course_points_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/course_points_num", 10,
            std::bind(&SafetyEvaluatorNode::coursePointsCallback, this, std::placeholders::_1));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SafetyEvaluatorNode::timerCallback, this));
    }

private:
    float calculateSpeedReduction() const
    {
        if (close_points_num_ > close_lim_) {
            return 0.0f;
        }
        if (course_points_num_ > course_lim_) {
            return 0.5f;
        }
        return 1.0f;
    }

    void cmdRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_raw_ = *msg;
    }

    void closePointsCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        close_points_num_ = static_cast<int>(msg->data);
    }

    void coursePointsCallback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        course_points_num_ = static_cast<int>(msg->data);
    }

    void timerCallback()
    {
        geometry_msgs::msg::Twist cmd_vel;
        const float reduction_ratio = calculateSpeedReduction();
        cmd_vel.linear.x = cmd_raw_.linear.x * reduction_ratio;
        cmd_vel.angular.z = cmd_raw_.angular.z * reduction_ratio;
        cmd_vel_pub_->publish(cmd_vel);
    }

    int close_lim_;
    int course_lim_;
    int close_points_num_;
    int course_points_num_;
    geometry_msgs::msg::Twist cmd_raw_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_raw_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr close_points_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr course_points_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyEvaluatorNode>());
    rclcpp::shutdown();
    return 0;
}