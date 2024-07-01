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

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class TrajectoryVisualizer : public rclcpp::Node
{
public:
  explicit TrajectoryVisualizer(const rclcpp::NodeOptions & options) : Node("trajectory_visualizer", options)
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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_marker", 10);
    reference_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/reference_trajectory_marker", 10);
    candidate_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_trajectory_marker", 10);
  }

private:
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
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

      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
  }

  void reference_trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
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
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      marker_array.markers.push_back(marker);
    }

    reference_marker_pub_->publish(marker_array);
  }

  void trajectory_candidateCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
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
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr reference_trajectory_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_candidate_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reference_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<TrajectoryVisualizer>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
