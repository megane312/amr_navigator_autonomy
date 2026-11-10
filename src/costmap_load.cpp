#include <memory>

#include <rclcpp/rclcpp.hpp>

class CostmapLoadNode : public rclcpp::Node
{
public:
  CostmapLoadNode()
  : Node("costmap")
  {
    RCLCPP_WARN(get_logger(), "costmap_load is a placeholder in ROS2 Jazzy; Nav2 costmap integration is not enabled in this workspace.");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapLoadNode>());
  rclcpp::shutdown();
  return 0;
}