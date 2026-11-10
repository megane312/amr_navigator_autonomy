#include "amr_navigator/odom_to_pose.hpp"

#include <atomic>

namespace amr_navigator
{

OdomToPoseNode::OdomToPoseNode()
: Node("odom_to_pose")
{
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&OdomToPoseNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Listening to /odom and publishing /pose");
}

void OdomToPoseNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header = msg->header;
    pose_stamped_msg.pose = msg->pose.pose;
    pose_pub_->publish(pose_stamped_msg);

    static std::atomic<int> pose_count{0};
    const int current_count = ++pose_count;
    if (current_count <= 5 || (current_count % 20) == 0) {
        RCLCPP_INFO(
            get_logger(),
            "Forwarded /odom -> /pose #%d x=%.3f y=%.3f",
            current_count,
            pose_stamped_msg.pose.position.x,
            pose_stamped_msg.pose.position.y);
    }
}

}  // namespace amr_navigator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<amr_navigator::OdomToPoseNode>());
    rclcpp::shutdown();
    return 0;
}