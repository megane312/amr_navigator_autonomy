#include "amr_navigator/waypoint_recorder.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace amr_navigator
{

WaypointRecorderNode::WaypointRecorderNode()
: Node("waypoint_recorder"), waypoint_interval_(0.25), idx_(0), saved_count_(0), has_last_point_(false)
{
    declare_parameter<std::string>("pose_topic", "/pose");
    declare_parameter<std::string>("waypoint_file", "./waypoints.txt");
    declare_parameter<double>("waypoint_interval", 0.25);
    declare_parameter<std::string>("frame_id", "world");

    get_parameter("waypoint_file", waypoint_filename_);
    get_parameter("waypoint_interval", waypoint_interval_);
    get_parameter("frame_id", marker_frame_id_);

    const std::filesystem::path waypoint_path(waypoint_filename_);
    const std::filesystem::path parent_path = waypoint_path.parent_path();
    if (!parent_path.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent_path, ec);
        if (ec) {
            RCLCPP_WARN(
                get_logger(),
                "Failed to create waypoint directory '%s': %s",
                parent_path.c_str(), ec.message().c_str());
        }
    }

    std::ofstream init_file(waypoint_filename_.c_str(), std::ios::app);
    if (!init_file.is_open()) {
        RCLCPP_ERROR(
            get_logger(),
            "Unable to open waypoint file for writing: %s",
            waypoint_filename_.c_str());
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/waypoint_marker", 10);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        get_parameter("pose_topic").as_string(), 10,
        std::bind(&WaypointRecorderNode::poseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "Ready to record the waypoints. pose_topic=%s waypoint_file=%s interval=%.3f",
        get_parameter("pose_topic").as_string().c_str(),
        waypoint_filename_.c_str(),
        waypoint_interval_);
}

void WaypointRecorderNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if (has_last_point_) {
        const double distance = std::sqrt(
            std::pow(last_point_[0] - msg->pose.position.x, 2) +
            std::pow(last_point_[1] - msg->pose.position.y, 2) +
            std::pow(last_point_[2] - msg->pose.position.z, 2));

        if (distance < waypoint_interval_) {
            return;
        }
    }

    if (waypoint_interval_ <= 0.0) {
        return;
    }

    last_point_[0] = msg->pose.position.x;
    last_point_[1] = msg->pose.position.y;
    last_point_[2] = msg->pose.position.z;
    last_point_[3] = msg->pose.orientation.x;
    last_point_[4] = msg->pose.orientation.y;
    last_point_[5] = msg->pose.orientation.z;
    last_point_[6] = msg->pose.orientation.w;
    has_last_point_ = true;

    std::ofstream record_file(waypoint_filename_.c_str(), std::ios::app);
    if (!record_file.is_open()) {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to write waypoint to %s",
            waypoint_filename_.c_str());
        return;
    }
    std::stringstream wp;
    for (size_t i = 0; i < last_point_.size(); ++i) {
        wp << last_point_[i];
        if (i + 1 != last_point_.size()) {
            wp << ",";
        }
    }
    record_file << wp.str() << std::endl;
    record_file.flush();
    ++saved_count_;

    RCLCPP_INFO(
        get_logger(),
        "Saved waypoint #%d to %s",
        saved_count_,
        waypoint_filename_.c_str());

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_id_;
    marker.header.stamp = now();
    marker.ns = "waypoints";
    marker.id = idx_++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_pub_->publish(marker);
}

}  // namespace amr_navigator

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<amr_navigator::WaypointRecorderNode>());
    rclcpp::shutdown();
    return 0;
}