// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under #include <memory>the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_lidar_simulator.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware_practice_lidar_simulator
{

SampleNode::SampleNode() : Node("simple_lidar_simulator")
{
  publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/simulator/ground_truth/lidar0/pointcloud", 10);
  pose_subscriber_ = create_subscription<PoseStamped>(
    "/simulator/ground_truth/pose", 10, std::bind(&SampleNode::pose_callback, this, std::placeholders::_1));
  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });

  // 物体の中心位置リストをCSVから読み取って初期化
  const auto pkg_path = ament_index_cpp::get_package_share_directory("autoware_practice_lidar_simulator");
  object_centers_ = load_object_centers_from_csv(pkg_path + "/config/object_centers.csv");}

void SampleNode::pose_callback(const PoseStamped::SharedPtr msg)
{
  last_pose_ = msg->pose;
}
std::vector<std::pair<float, float>> SampleNode::load_object_centers_from_csv(const std::string & file_path)
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

  while (std::getline(file, line)) {
    RCLCPP_INFO(this->get_logger(), "Processing line: %s", line.c_str());
    std::istringstream line_stream(line);
    std::string x_str, y_str;

    if (std::getline(line_stream, x_str, ',') && std::getline(line_stream, y_str, ',')) {
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
  for (const auto & center : object_centers_) {
    auto object_cloud = create_object_point_cloud(center.first, center.second, 3.0, 2.0, 0.5);
    *cloud += *object_cloud;
  }
  // 車両から半径10m以内の点群を抽出
  auto filtered_cloud = filter_points_within_radius(cloud, 10.0);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*filtered_cloud, output);
  output.header.frame_id = "map";
  output.header.stamp = this->now();

  publisher_->publish(output);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleNode::create_object_point_cloud(
  float x_center, float y_center, float width, float height, float resolution)
{
  auto cloud = std::make_shared<PointCloudXYZ>();

  float x_start = x_center - width / 2;
  float x_end = x_center + width / 2;
  float y_start = y_center - height / 2;
  float y_end = y_center + height / 2;

  // 上辺と下辺
  for (float x = x_start; x <= x_end; x += resolution) {
    cloud->points.emplace_back(pcl::PointXYZ{x, y_start, 0.0});
    cloud->points.emplace_back(pcl::PointXYZ{x, y_end, 0.0});
  }

  // 左辺と右辺
  for (float y = y_start; y <= y_end; y += resolution) {
    cloud->points.emplace_back(pcl::PointXYZ{x_start, y, 0.0});
    cloud->points.emplace_back(pcl::PointXYZ{x_end, y, 0.0});
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr SampleNode::filter_points_within_radius(PointCloudXYZ::Ptr cloud, float radius)
{
  auto filtered_cloud = std::make_shared<PointCloudXYZ>();
  for (const auto & point : cloud->points) {
    float x = point.x - last_pose_.position.x;
    float y = point.y - last_pose_.position.y;
    if (sqrt(x * x + y * y) <= radius) {
      filtered_cloud->points.push_back(point);
    }
  }
  return filtered_cloud;
}

}  // namespace autoware_practice_lidar_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_lidar_simulator::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
