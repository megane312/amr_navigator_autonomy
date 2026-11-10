#include "amr_navigator/draw_line.hpp"

#include <chrono>
#include <functional>
#include <cmath>

namespace amr_navigator
{

PolygonPublisher::PolygonPublisher()
: Node("pub"), width_(0.5), ratio_(4.0), turning_radius_(0.0), linear_velocity_(0.0)
{
    pub_ = create_publisher<visualization_msgs::msg::Marker>("course", 1);
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "radius_vel", 10,
        std::bind(&PolygonPublisher::callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(
        std::chrono::milliseconds(40),
        std::bind(&PolygonPublisher::timerCallback, this));
}

void PolygonPublisher::callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) {
        return;
    }
    turning_radius_.store(msg->data[0]);
    linear_velocity_.store(msg->data[1]);
}

std::vector<geometry_msgs::msg::Point32> PolygonPublisher::define_half_circle(
    double linear_velocity, double turning_radius, double width, int num_points)
{
    const double cx = 0.0;
    const double cy = -turning_radius;
    const double cz = 0.0;
    const double r1 = turning_radius - width / 2.0;
    const double r2 = turning_radius + width / 2.0;
    const double start_angle = 0.0;
    const double end_angle = (linear_velocity * ratio_) / turning_radius;

    std::vector<geometry_msgs::msg::Point32> points;
    for (int i = 0; i <= num_points; ++i) {
        const double a = start_angle + (end_angle - start_angle) * static_cast<double>(i) / num_points;
        geometry_msgs::msg::Point32 circle_point;
        circle_point.x = cx + r2 * std::sin(a);
        circle_point.y = -(cy + r2 * std::cos(a));
        circle_point.z = cz;
        points.push_back(circle_point);
    }
    for (int i = num_points; i >= 0; --i) {
        const double a = start_angle + (end_angle - start_angle) * static_cast<double>(i) / num_points;
        geometry_msgs::msg::Point32 circle_point;
        circle_point.x = cx + r1 * std::sin(a);
        circle_point.y = -(cy + r1 * std::cos(a));
        circle_point.z = cz;
        points.push_back(circle_point);
    }
    return points;
}

std::vector<geometry_msgs::msg::Point32> PolygonPublisher::define_rectangle(double linear_velocity, double width)
{
    std::vector<geometry_msgs::msg::Point32> points;
    geometry_msgs::msg::Point32 point;

    point.x = 0.0;
    point.y = -width / 2.0;
    point.z = 0.0;
    points.push_back(point);

    point.y = width / 2.0;
    points.push_back(point);

    point.x = linear_velocity * ratio_;
    points.push_back(point);

    point.y = -width / 2.0;
    points.push_back(point);

    return points;
}

visualization_msgs::msg::Marker PolygonPublisher::makeMarker(double linear_velocity, double turning_radius, double width)
{
    std::vector<geometry_msgs::msg::Point32> points;
    if (turning_radius != 0.0) {
        points = define_half_circle(linear_velocity, turning_radius, width, 20);
    } else {
        points = define_rectangle(linear_velocity, width);
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = now();
    marker.ns = "course";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.6;
    marker.color.b = 1.0;

    for (const auto & point32 : points) {
        geometry_msgs::msg::Point point;
        point.x = point32.x;
        point.y = point32.y;
        point.z = point32.z;
        marker.points.push_back(point);
    }
    if (!marker.points.empty()) {
        marker.points.push_back(marker.points.front());
    }
    return marker;
}

void PolygonPublisher::timerCallback()
{
    pub_->publish(makeMarker(linear_velocity_.load(), turning_radius_.load(), width_));
}

}  // namespace amr_navigator

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<amr_navigator::PolygonPublisher>());
    rclcpp::shutdown();
    return 0;
}