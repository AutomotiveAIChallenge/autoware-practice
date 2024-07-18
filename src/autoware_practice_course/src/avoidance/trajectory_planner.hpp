// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AVOIDANCE__TRAJECTORY_PLANNER_HPP_
#define AVOIDANCE__TRAJECTORY_PLANNER_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_practice_msgs/msg/float_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

// フリー関数としての + 演算子のオーバーロードの宣言

namespace autoware_practice_course
{
geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
  TrajectoryPlannerNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using Odometry = nav_msgs::msg::Odometry;
  using Point = geometry_msgs::msg::Point;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Quaternion = geometry_msgs::msg::Quaternion;

  void on_timer();
  void update_current_state(const Odometry & msg);
  void update_reference_trajectory(const Trajectory & msg);
  void update_pointcloud(const PointCloud2 & msg);
  void create_trajectory();
  std::vector<Trajectory> create_trajectory_set();
  TrajectoryPoint calculate_target_trajectory_point();
  std::vector<std::vector<float>> create_costmap();
  Trajectory evaluate_trajectory(
    const std::vector<Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap);
  std::vector<TrajectoryPoint> create_target_state_set();
  std::vector<Point> bezierInterpolate(const Point & p0, const Point & p1, Eigen::Vector3d m0, Eigen::Vector3d m1);
  double quaternionToInclination(Eigen::Quaterniond q);
  Eigen::Vector3d quaternionToVector(Eigen::Quaterniond q);
  Eigen::Vector3d pointToVector3d(const geometry_msgs::msg::Point & point);
  geometry_msgs::msg::Point vector3dToPoint(const Eigen::Vector3d & vector);
  Eigen::Quaterniond vectorToQuaternion(const Eigen::Vector3d & start, const Eigen::Vector3d & end);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_candidate_;
  rclcpp::Publisher<autoware_practice_msgs::msg::FloatGrid>::SharedPtr pub_costmap_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;

  double grid_resolution_;         // コストマップの解像度（メートル）
  double grid_width_;              // 目標状態の間隔（メートル）
  double grid_height_;             // 目標状態までのインデックス
  int state_num_;                  // ベジエ曲線による補間を分割する点の数
  double target_interval_;         // ベジエ曲線の端点から制御点までの距離（メートル）
  int target_index_;               // コストマップの幅（メートル）
  int num_points_;                 // コストマップの高さ（メートル）
  double control_point_distance_;  // 目標状態の数

  PointCloud2 pointcloud_;
  Point current_position_;
  Quaternion current_orientation_;
  double current_velocity_;
  Trajectory reference_trajectory_;
  Trajectory best_trajectory_;
  Trajectory trajectory_candidate_;
  std::vector<std::vector<float>> costmap_;

  bool current_state_initialized_ = false;
  bool reference_trajectory_initialized_ = false;
  bool pointcloud_initialized_ = false;
};

}  // namespace autoware_practice_course

#endif  // AVOIDANCE__TRAJECTORY_PLANNER_HPP_
