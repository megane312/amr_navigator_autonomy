#ifndef AMR_NAVIGATOR__ODOM_TO_POSE_HPP_
#define AMR_NAVIGATOR__ODOM_TO_POSE_HPP_

#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace amr_navigator
{

class OdomToPoseNode : public rclcpp::Node
{
public:
	OdomToPoseNode();

private:
	void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

}  // namespace amr_navigator

#endif  // AMR_NAVIGATOR__ODOM_TO_POSE_HPP_