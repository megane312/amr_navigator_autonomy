#ifndef AMR_NAVIGATOR__WAYPOINT_RECORDER_HPP_
#define AMR_NAVIGATOR__WAYPOINT_RECORDER_HPP_

#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <filesystem>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace amr_navigator
{

class WaypointRecorderNode : public rclcpp::Node
{
public:
	WaypointRecorderNode();

private:
	void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

	std::array<double, 7> last_point_{};
	std::string waypoint_filename_;
	std::string marker_frame_id_;
	double waypoint_interval_;
	int idx_;
	int saved_count_;
	bool has_last_point_;

	rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace amr_navigator

#endif  // AMR_NAVIGATOR__WAYPOINT_RECORDER_HPP_