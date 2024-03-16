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

#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include "interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

namespace autoware_practice_simulator
{

using ControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::GearReport;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::VelocityReport;

struct VehicleController
{
public:
  explicit VehicleController(rclcpp::Node & node);
  VehicleInput input() const;
  void update(double dt, const VehicleState & state);
  void publish(const rclcpp::Time & stamp);

private:
  void on_command(const ControlCommand & msg);
  void on_gear(const GearCommand & msg);
  rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;

  VehicleInput input_;
  VehicleState state_;
  double speed_;
  double accel_;
  double steer_;
  GearReport::_report_type gear_;
};

}  // namespace autoware_practice_simulator

#endif  // CONTROLLER_HPP_
