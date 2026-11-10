#include <cmath>
#include <memory>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class DetectionClosePointNode : public rclcpp::Node
{
public:
    DetectionClosePointNode()
    : Node("pcl_change_detector"), linear_velocity_(0.0), turning_radius_(0.0)
    {
        pub_close_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("close_points", 10);
        pub_close_points_num_ = create_publisher<std_msgs::msg::Float32>("close_points_num", 10);

        sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&DetectionClosePointNode::cloudCB, this, std::placeholders::_1));
        sub_radius_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "radius_vel", 10,
            std::bind(&DetectionClosePointNode::radiusCB, this, std::placeholders::_1));
    }

private:
    void radiusCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 2) {
            turning_radius_ = msg->data[0];
            linear_velocity_ = msg->data[1];
        }
    }

    void cloudCB(const sensor_msgs::msg::PointCloud2::SharedPtr input)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input, *cloud_input);

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_input);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*cloud_input);

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);
        pass.setInputCloud(cloud_input);
        pass.filter(*cloud_input);

        const double width = 0.5;
        const double ratio = 3.0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (turning_radius_ != 0.0) {
            const double center_angle = (linear_velocity_ * ratio) / turning_radius_;
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            if (turning_radius_ > 0) {
                boxFilter.setMin(Eigen::Vector4f(0, -width / 2, 0.1, 1.0));
                boxFilter.setMax(Eigen::Vector4f(turning_radius_ * std::sin(center_angle), turning_radius_, 2.0, 1.0));
            } else {
                boxFilter.setMin(Eigen::Vector4f(0, turning_radius_, 0.1, 1.0));
                boxFilter.setMax(Eigen::Vector4f(turning_radius_ * std::sin(center_angle), width / 2, 2.0, 1.0));
            }
            boxFilter.setInputCloud(cloud_input);
            boxFilter.filter(*cloud_input);

            const double inner_radius = std::abs(turning_radius_) - width / 2;
            const double outer_radius = std::abs(turning_radius_) + width / 2;
            for (const auto & point : cloud_input->points) {
                const double point_distance = std::sqrt(point.x * point.x + (point.y - turning_radius_) * (point.y - turning_radius_));
                if (point_distance <= outer_radius && point_distance >= inner_radius) {
                    filtered_cloud->points.push_back(point);
                }
            }
        } else {
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            boxFilter.setMin(Eigen::Vector4f(0.0, -width / 2, 0.0, 1.0));
            boxFilter.setMax(Eigen::Vector4f(linear_velocity_ * ratio, width / 2, 2.0, 1.0));
            boxFilter.setInputCloud(cloud_input);
            boxFilter.filter(*filtered_cloud);
        }

        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "livox_frame";
        output.header.stamp = input->header.stamp;
        pub_close_points_->publish(output);

        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(filtered_cloud->points.size());
        pub_close_points_num_->publish(msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_close_points_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_close_points_num_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_radius_;
    double linear_velocity_;
    double turning_radius_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionClosePointNode>());
    rclcpp::shutdown();
    return 0;
}