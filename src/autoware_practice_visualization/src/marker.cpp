//  Copyright 2023 The Autoware Contributors
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "marker.hpp"

#include <memory>
namespace autoware_practice_visualization
{

TrajectoryVisualizer::TrajectoryVisualizer(const rclcpp::NodeOptions & options) : Node("marker", options)
{
  trajectory_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory", 10,
    std::bind(&TrajectoryVisualizer::trajectoryCallback, this, std::placeholders::_1));
  reference_trajectory_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/trajectory_loader/trajectory", 10,
    std::bind(&TrajectoryVisualizer::reference_trajectoryCallback, this, std::placeholders::_1));
  trajectory_candidate_sub_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "/planning/scenario_planning/trajectory_candidate", 10,
    std::bind(&TrajectoryVisualizer::trajectory_candidateCallback, this, std::placeholders::_1));
  costmap_sub_ = this->create_subscription<autoware_practice_msgs::msg::FloatGrid>(
    "/planning/scenario_planning/costmap", 10,
    std::bind(&TrajectoryVisualizer::costmapCallback, this, std::placeholders::_1));

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/trajectory_marker", 10);
  reference_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/reference_trajectory_marker", 10);
  candidate_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/candidate_trajectory_marker", 10);
  costmap_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/costmap_marker", 10);
}

void TrajectoryVisualizer::trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < msg->points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.header.frame_id = "map";  // 適切なフレームIDを設定
    marker.ns = "trajectory";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = msg->points[i].pose.position;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

void TrajectoryVisualizer::reference_trajectoryCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < msg->points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.header.frame_id = "map";  // 適切なフレームIDを設定
    marker.ns = "trajectory";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = msg->points[i].pose.position;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }

  reference_marker_pub_->publish(marker_array);
}

void TrajectoryVisualizer::trajectory_candidateCallback(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < msg->points.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.header.frame_id = "map";  // 適切なフレームIDを設定
    marker.ns = "trajectory";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position = msg->points[i].pose.position;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker_array.markers.push_back(marker);
  }

  candidate_marker_pub_->publish(marker_array);
}

void TrajectoryVisualizer::costmapCallback(const autoware_practice_msgs::msg::FloatGrid::SharedPtr msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  float resolution = 1.0;  // コストマップの解像度（必要に応じて設定）

  for (size_t y = 0; y < msg->height; ++y) {
    for (size_t x = 0; x < msg->width; ++x) {
      size_t index = y + x * msg->height;
      if (index >= msg->data.size()) continue;  // 安全対策
      float cost = msg->data[index];
      if (cost > 100.0) {
        RCLCPP_INFO(this->get_logger(), "index: %ld cost: %f", index, cost);  // コスト値の表示（デバッグ用
      }

      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";  // 適切なフレームIDを設定
      marker.ns = "costmap";
      marker.id = index;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position.x = x * resolution;
      marker.pose.position.y = y * resolution - 50;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = resolution;
      marker.scale.y = resolution;
      marker.scale.z = 0.1;

      marker.color.r = cost / 1000;  // コスト値に基づく色の設定（適宜調整）
      marker.color.g = 1.0 - cost / 1000;
      marker.color.b = 0.0;
      marker.color.a = 0.5;

      marker_array.markers.push_back(marker);
    }
  }

  candidate_marker_pub_->publish(marker_array);
}

}  // namespace autoware_practice_visualization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<autoware_practice_visualization::TrajectoryVisualizer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
