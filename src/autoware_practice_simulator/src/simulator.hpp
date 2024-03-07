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

#include "geometry.hpp"
#include "kinematics.hpp"

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

#include <optional>

namespace autoware_practice_simulator
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;

class Simulator : public rclcpp::Node
{
public:
  explicit Simulator(const rclcpp::NodeOptions & options);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<PoseStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;

private:
  void on_command(const AckermannControlCommand & msg);
  void on_gear(const GearCommand & msg);
  void on_timer();
  std::optional<Kinematics> kinematics_;
};

}  // namespace autoware_practice_simulator

#endif  // SIMULATOR_HPP_
