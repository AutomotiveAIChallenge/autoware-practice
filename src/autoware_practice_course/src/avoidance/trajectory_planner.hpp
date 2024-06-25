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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

// フリー関数としての + 演算子のオーバーロードの宣言

namespace autoware_practice_course
{
geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

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
  Trajectory create_trajectory();
  std::vector<Trajectory> create_trajectory_set();
  TrajectoryPoint calculate_target_trajectory_point();
  std::vector<std::vector<float>> create_costmap();
  Trajectory evaluate_trajectory(
    const std::vector<Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap);
  std::vector<TrajectoryPoint> create_target_state_set();
  std::vector<Point> hermiteInterpolate(const Point & p0, const Point & p1, double m0, double m1, int numPoints);
  double quaternionToInclination(Eigen::Quaterniond q);
  Eigen::Vector3d pointToVector3d(const geometry_msgs::msg::Point & point);
  geometry_msgs::msg::Point vector3dToPoint(const Eigen::Vector3d & vector);
  Eigen::Quaterniond vectorToQuaternion(const Eigen::Vector3d & start, const Eigen::Vector3d & end);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;

  double GRID_RESOLUTION_;
  double GRID_WIDTH_;
  double GRID_HEIGHT_;
  PointCloud2 pointcloud_;
  Point current_position_;
  Quaternion current_orientation_;
  double current_velocity_;
  Trajectory reference_trajectory_;
  int state_num_;

  bool current_state_initialized_ = false;
  bool reference_trajectory_initialized_ = false;
  bool pointcloud_initialized_ = false;
};

}  // namespace autoware_practice_course

#endif  // AVOIDANCE__TRAJECTORY_PLANNER_HPP_
