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

#ifndef MARKER_HPP_
#define MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_practice_msgs/msg/float_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware_practice_visualization
{
class TrajectoryVisualizer : public rclcpp::Node
{
public:
  explicit TrajectoryVisualizer(const rclcpp::NodeOptions & options);

private:
  void trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void reference_trajectoryCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void trajectory_candidateCallback(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void costmapCallback(const autoware_practice_msgs::msg::FloatGrid::SharedPtr msg);

  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr reference_trajectory_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_candidate_sub_;
  rclcpp::Subscription<autoware_practice_msgs::msg::FloatGrid>::SharedPtr costmap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reference_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr costmap_marker_pub_;
};

}  // namespace autoware_practice_visualization

#endif  // MARKER_HPP_
