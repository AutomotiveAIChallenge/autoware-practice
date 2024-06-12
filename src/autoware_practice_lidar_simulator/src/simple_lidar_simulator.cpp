#include "simple_lidar_simulator.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace autoware_practice_lidar_simulator
{

SampleNode::SampleNode() : Node("simple_lidar_simulator")
{
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    timer_ = create_wall_timer(
        //std::chrono::milliseconds(500),
        std::chrono::seconds(1), 
        std::bind(&LidarNode::timer_callback, this));
}

void SampleNode::timer_callback()
{
    // 物体の中心位置 (x_center, y_center)
    float x_center = 7.0;  // 車両の前方1m
    float y_center = 0.0;  // 車両の中心線上

    // 1m x 2mの物体の点群を生成
    auto cloud = create_object_point_cloud(x_center, y_center, 3.0, 2.0, 0.5);

    // 車両から半径3m以内の点群を抽出
    auto filtered_cloud = filter_points_within_radius(cloud, 10.0);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = this->now();

    publisher_->publish(output);
}

PointCloudXYZ::Ptr SampleNode::create_object_point_cloud(float x_center, float y_center, float width, float height, float resolution)
{
    auto cloud = std::make_shared<PointCloudXYZ>();

    float x_start = x_center - width / 2;
    float x_end = x_center + width / 2;
    float y_start = y_center - height / 2;
    float y_end = y_center + height / 2;

    // 上辺と下辺
    for (float x = x_start; x <= x_end; x += resolution)
    {
        cloud->points.emplace_back(pcl::PointXYZ{x, y_start, 0.0});
        cloud->points.emplace_back(pcl::PointXYZ{x, y_end, 0.0});
    }

    // 左辺と右辺
    for (float y = y_start; y <= y_end; y += resolution)
    {
        cloud->points.emplace_back(pcl::PointXYZ{x_start, y, 0.0});
        cloud->points.emplace_back(pcl::PointXYZ{x_end, y, 0.0});
    }

    return cloud;
}

PointCloudXYZ::Ptr SampleNode::filter_points_within_radius(PointCloudXYZ::Ptr cloud, float radius)
{
    auto filtered_cloud = std::make_shared<PointCloudXYZ>();
    for (const auto& point : cloud->points)
    {
        if (sqrt(point.x * point.x + point.y * point.y) <= radius)
        {
            filtered_cloud->points.push_back(point);
        }
    }
    return filtered_cloud;
}

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared< autoware_practice_lidar_simulator::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
