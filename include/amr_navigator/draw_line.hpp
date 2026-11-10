#ifndef AMR_NAVIGATOR__DRAW_LINE_HPP_
#define AMR_NAVIGATOR__DRAW_LINE_HPP_

#include <atomic>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace amr_navigator
{

class PolygonPublisher : public rclcpp::Node
{
public:
    PolygonPublisher();

private:
    void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void timerCallback();
    std::vector<geometry_msgs::msg::Point32> define_half_circle(double linear_velocity, double turning_radius, double width, int num_points);
    std::vector<geometry_msgs::msg::Point32> define_rectangle(double linear_velocity, double width);
    visualization_msgs::msg::Marker makeMarker(double linear_velocity, double turning_radius, double width);

    const double width_;
    const double ratio_;
    std::atomic<double> turning_radius_;
    std::atomic<double> linear_velocity_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_navigator

#endif  // AMR_NAVIGATOR__DRAW_LINE_HPP_