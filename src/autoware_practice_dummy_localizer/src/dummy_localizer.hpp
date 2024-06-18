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

#ifndef DUMMY_LOCALIZER_HPP_
#define DUMMY_LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace autoware_practice_course
{
class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Odometry = nav_msgs::msg::Odometry;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Pose = geometry_msgs::msg::Pose;
  using Twist = geometry_msgs::msg::Twist;

  void pose_callback(const PoseStamped::SharedPtr msg);
  void twist_callback(const TwistStamped::SharedPtr msg);
  void publish_odometry();

  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscriber_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_subscriber_;
  Pose last_pose_;
  Twist last_twist_;
  bool has_pose_;
  bool has_twist_;
};
}  // namespace autoware_practice_course
#endif  // DUMMY_LOCALIZER_HPP_
