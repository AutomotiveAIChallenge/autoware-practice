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

#ifndef VELOCITY_PLANNING__LONGITUDINAL_CONTROLLER_HPP_
#define VELOCITY_PLANNING__LONGITUDINAL_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using Odometry = nav_msgs::msg::Odometry;
  using Point = geometry_msgs::msg::Point;

  void on_timer();
  void update_target_velocity(const Trajectory & msg);
  void update_current_state(const Odometry & msg);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematic_state_;
  Point current_pose_;
  double current_velocity_;
  double target_velocity_;
  double kp_;
};

}  // namespace autoware_practice_course

#endif  // VELOCITY_PLANNING__LONGITUDINAL_CONTROLLER_HPP_
