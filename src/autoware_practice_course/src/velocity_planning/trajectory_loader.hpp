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

#ifndef VELOCITY_PLANNING__TRAJECTORY_LOADER_HPP_
#define VELOCITY_PLANNING__TRAJECTORY_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>

#include <string>

namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

  void on_timer();
  void load_path(const std::string & file_path);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  Trajectory trajectory_;
};

}  // namespace autoware_practice_course

#endif  // VELOCITY_PLANNING__TRAJECTORY_LOADER_HPP_
