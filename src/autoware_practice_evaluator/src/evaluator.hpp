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

#ifndef EVALUATOR_HPP_
#define EVALUATOR_HPP_

#include "condition.hpp"

#include <rclcpp/rclcpp.hpp>
#include <autoware_practice_msgs/msg/judge_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <memory>
#include <vector>

namespace autoware_practice_evaluator
{

class Evaluator : public rclcpp::Node
{
public:
  explicit Evaluator(const rclcpp::NodeOptions & options);

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using VehiclePath = nav_msgs::msg::Path;
  using JudgeStatus = autoware_practice_msgs::msg::JudgeStatus;

  void on_pose(const PoseStamped & msg);
  void on_path(const VehiclePath & msg);
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<VehiclePath>::SharedPtr sub_path_;
  rclcpp::Publisher<JudgeStatus>::SharedPtr pub_result_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;

  Condition::UniquePtr condition_tree_;
  std::vector<Condition *> condition_list_;
  JudgeInput data_;
};

}  // namespace autoware_practice_evaluator

#endif  // EVALUATOR_HPP_
