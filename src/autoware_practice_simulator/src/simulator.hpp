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

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include "controller.hpp"
#include "kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace autoware_practice_simulator
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class Simulator : public rclcpp::Node
{
public:
  explicit Simulator(const rclcpp::NodeOptions & options);

private:
  void on_timer_sim();
  void on_timer_pub();
  void publish(const rclcpp::Time & stamp);

  std::unique_ptr<VehicleKinematics> kinematics_;
  std::unique_ptr<VehicleController> controller_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_sim_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;

  double rate_sim_;
  double rate_pub_;
};

}  // namespace autoware_practice_simulator

#endif  // SIMULATOR_HPP_
