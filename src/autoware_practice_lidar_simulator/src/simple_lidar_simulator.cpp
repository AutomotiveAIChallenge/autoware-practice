#include "simple_lidar_simulator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace autoware_practice_lidar_simulator
{

SampleNode::SampleNode() : Node("simple_lidar_simulator")
{
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
    const auto period = rclcpp::Rate(10).period();
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
    

    // 物体の中心位置リストをCSVから読み取って初期化
    object_centers_ = read_object_centers_from_csv("src/autoware_practice_lidar_simulator/config/object_centers.csv");
}

std::vector<std::pair<float, float>> SampleNode::read_object_centers_from_csv(const std::string& file_path)
{
    std::vector<std::pair<float, float>> centers;
    std::ifstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
        return centers;
    }

    std::string line;
    std::getline(file, line);
    RCLCPP_INFO(this->get_logger(), "Skipping header: %s", line.c_str());

    while (std::getline(file, line))
    {
        RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());
        std::istringstream line_stream(line);
        std::string x_str, y_str;

        if (std::getline(line_stream, x_str, ',') && std::getline(line_stream, y_str, ','))
        {
            float x = std::stof(x_str);
            float y = std::stof(y_str);
            centers.emplace_back(x, y);
        }
    }
    file.close();

    return centers;
}


void SampleNode::on_timer()
{
    auto cloud = std::make_shared<PointCloudXYZ>();

    // 各物体の点群を生成
    for (const auto& center : object_centers_)
    {
        auto object_cloud = create_object_point_cloud(center.first, center.second, 3.0, 2.0, 0.5);
        *cloud += *object_cloud;
    }
    // 車両から半径3m以内の点群を抽出
    auto filtered_cloud = filter_points_within_radius(cloud, 10.0);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud, output);
    output.header.frame_id = "map";
    output.header.stamp = this->now();

    publisher_->publish(output);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleNode::create_object_point_cloud(float x_center, float y_center, float width, float height, float resolution)
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

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleNode::filter_points_within_radius(PointCloudXYZ::Ptr cloud, float radius)
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
