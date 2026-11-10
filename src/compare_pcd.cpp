#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include <Eigen/Dense>

#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class ComparePCDNode : public rclcpp::Node
{
public:
    ComparePCDNode()
    : Node("pcl_change_detector"),
        cloud_map_(new pcl::PointCloud<pcl::PointXYZ>),
        octree_(0.5f),
        turning_radius_(0.0),
        linear_velocity_(0.0),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
    {
        declare_parameter<std::string>("pcd_file_path", "");
        std::string pcd_file_path;
        get_parameter("pcd_file_path", pcd_file_path);
        if (pcd_file_path.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to get param pcd_file_path");
            return;
        }

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud_map_) == -1) {
            RCLCPP_ERROR(get_logger(), "Couldn't read the file cloud_map.pcd");
            return;
        }

        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_map_);
        sor.setLeafSize(0.25f, 0.25f, 0.25f);
        sor.filter(*cloud_map_);

        pub_diff_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("diff_points", 10);
        pub_course_points_ = create_publisher<sensor_msgs::msg::PointCloud2>("course_points", 10);
        pub_course_points_num_ = create_publisher<std_msgs::msg::Float32>("course_points_num", 10);

        sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/aligned_points", 10,
            std::bind(&ComparePCDNode::cloudCB, this, std::placeholders::_1));
        sub_radius_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "radius_vel", 10,
            std::bind(&ComparePCDNode::radiusCB, this, std::placeholders::_1));
    }

private:
    Eigen::Affine3f lookupTransform()
    {
        const auto transform_stamped = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero);
        return tf2::transformToEigen(transform_stamped.transform).cast<float>();
    }

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

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, 2.0);
        pass.setInputCloud(cloud_input);
        pass.filter(*cloud_input);

        octree_.setInputCloud(cloud_map_);
        octree_.addPointsFromInputCloud();
        octree_.switchBuffers();
        octree_.setInputCloud(cloud_input);
        octree_.addPointsFromInputCloud();

        std::vector<int> newPointIdxVector;
        octree_.getPointIndicesFromNewVoxels(newPointIdxVector);
        octree_.switchBuffers();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_diff(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto idx : newPointIdxVector) {
            if (idx >= 0 && static_cast<std::size_t>(idx) < cloud_input->points.size()) {
                cloud_diff->points.push_back(cloud_input->points[idx]);
            }
        }
        cloud_diff->width = cloud_diff->points.size();
        cloud_diff->height = 1;
        cloud_diff->is_dense = true;

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_diff, output);
        output.header.frame_id = "map";
        output.header.stamp = input->header.stamp;
        pub_diff_points_->publish(output);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        try {
            pcl::transformPointCloud(*cloud_diff, *transformed_cloud, lookupTransform());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF lookup failed: %s", ex.what());
            return;
        }

        const double width = 0.5;
        const double ratio = 8.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (turning_radius_ != 0.0) {
            const double center_angle = (linear_velocity_ * ratio) / turning_radius_;
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            if (turning_radius_ > 0) {
                boxFilter.setMin(Eigen::Vector4f(0, -width / 2, -0.1, 1.0));
                boxFilter.setMax(Eigen::Vector4f(turning_radius_ * std::sin(center_angle), turning_radius_, 2.0, 1.0));
            } else {
                boxFilter.setMin(Eigen::Vector4f(0, turning_radius_, -0.1, 1.0));
                boxFilter.setMax(Eigen::Vector4f(turning_radius_ * std::sin(center_angle), width / 2, 2.0, 1.0));
            }
            boxFilter.setInputCloud(transformed_cloud);
            boxFilter.filter(*transformed_cloud);

            const double inner_radius = std::abs(turning_radius_) - width / 2;
            const double outer_radius = std::abs(turning_radius_) + width / 2;
            for (const auto & point : transformed_cloud->points) {
                const double point_distance = std::sqrt(point.x * point.x + (point.y - turning_radius_) * (point.y - turning_radius_));
                if (point_distance <= outer_radius && point_distance >= inner_radius) {
                    filtered_cloud->points.push_back(point);
                }
            }
        } else {
            pcl::CropBox<pcl::PointXYZ> boxFilter;
            boxFilter.setMin(Eigen::Vector4f(0.0, -width / 2, 0.0, 1.0));
            boxFilter.setMax(Eigen::Vector4f(linear_velocity_ * ratio, width / 2, 2.0, 1.0));
            boxFilter.setInputCloud(transformed_cloud);
            boxFilter.filter(*filtered_cloud);
        }

        pcl::toROSMsg(*filtered_cloud, output);
        output.header.frame_id = "base_link";
        output.header.stamp = input->header.stamp;
        pub_course_points_->publish(output);

        std_msgs::msg::Float32 msg;
        msg.data = static_cast<float>(filtered_cloud->points.size());
        pub_course_points_num_->publish(msg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_diff_points_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_course_points_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_course_points_num_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_radius_;
    double turning_radius_;
    double linear_velocity_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ComparePCDNode>());
    rclcpp::shutdown();
    return 0;
}