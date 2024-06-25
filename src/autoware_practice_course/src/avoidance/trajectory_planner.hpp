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

namespace autoware_practice_course
{

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
  std::vector<TrajectoryPoint> create_target_state_set(const TrajectoryPoint & target_trajectory_point, int state_num);
  TrajectoryPoint calculate_target_trajectory_point();
  std::vector<std::vector<float>> create_costmap();
  Trajectory evaluate_trajectory(
    const std::vector<Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_pointcloud_;

  PointCloud2 pointcloud_;
  Point current_position_;
  Quaternion current_orientation_;
  double current_velocity_;
  Trajectory reference_trajectory_;
};

}  // namespace autoware_practice_course

#endif  // AVOIDANCE__TRAJECTORY_PLANNER_HPP_
