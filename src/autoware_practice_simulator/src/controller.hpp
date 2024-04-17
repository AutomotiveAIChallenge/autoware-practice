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

#include "kinematics.hpp"

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

class VehicleController
{
public:
  VehicleController(rclcpp::Node & node, const VehicleSpecs & specs, VehicleKinematics * kinematics);
  void update(double dt);
  void publish(const rclcpp::Time & stamp);
  VehicleSpecs specs() const { return specs_; }

private:
  void on_command(const ControlCommand & msg);
  void on_gear(const GearCommand & msg);
  rclcpp::Subscription<ControlCommand>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Publisher<GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;

  struct VehicleCommand;
  VehicleCommand create_command();

  VehicleKinematics * kinematics_;
  VehicleSpecs specs_;
  GearReport::_report_type gear_;
  double actual_speed_;
  double actual_accel_;
  double actual_steer_;
  double target_speed_;
  double target_accel_;
  double target_steer_;
  double heading_rate_;
};

}  // namespace autoware_practice_simulator

#endif  // CONTROLLER_HPP_
