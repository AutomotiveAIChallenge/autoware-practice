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

#include "controller.hpp"

#include <algorithm>

namespace autoware_practice_simulator
{

VehicleController::VehicleController(rclcpp::Node & node)
{
  using std::placeholders::_1;
  sub_control_ = node.create_subscription<ControlCommand>("~/command/control", rclcpp::QoS(1), std::bind(&VehicleController::on_command, this, _1));
  sub_gear_ = node.create_subscription<GearCommand>("~/command/gear", rclcpp::QoS(1), std::bind(&VehicleController::on_gear, this, _1));
  pub_gear_ = node.create_publisher<GearReport>("~/status/gear", rclcpp::QoS(1));
  pub_velocity_ = node.create_publisher<VelocityReport>("~/status/velocity", rclcpp::QoS(1));
  pub_steering_ = node.create_publisher<SteeringReport>("~/status/steering", rclcpp::QoS(1));

  input_ = VehicleInput{true, 0.0, 0.0, 0.0};
  speed_ = 0.0;
  accel_ = 0.0;
  steer_ = 0.0;
  gear_ = GearReport::DRIVE;
}

void VehicleController::on_command(const ControlCommand & msg)
{
  speed_ = msg.longitudinal.speed;
  accel_ = msg.longitudinal.acceleration;
  steer_ = msg.lateral.steering_tire_angle;
}

void VehicleController::on_gear(const GearCommand & msg)
{
  // clang-format off
  switch (msg.command) {
    case GearCommand::PARK:    gear_ = GearReport::PARK;    break;
    case GearCommand::NEUTRAL: gear_ = GearReport::NEUTRAL; break;
    case GearCommand::DRIVE:   gear_ = GearReport::DRIVE;   break;
    case GearCommand::REVERSE: gear_ = GearReport::REVERSE; break;
  }
  // clang-format on
}

VehicleInput VehicleController::input() const
{
  return input_;
}

void VehicleController::update(double dt, const VehicleState & state)
{
  state_ = state;

  if (gear_ == GearReport::PARK) {
    input_.parking = true;
    input_.accel = 0.0;
    input_.brake = 0.0;
    input_.steer = steer_;
    return;
  }

  if (gear_ == GearReport::NEUTRAL) {
    input_.parking = false;
    input_.accel = 0.0;
    input_.brake = 0.0;
    input_.steer = steer_;
    return;
  }

  if (gear_ == GearReport::DRIVE) {
    const auto speed_delta = (speed_ - state_.speed) / dt;
    const auto ideal_accel = std::max(0.0, +speed_delta);
    const auto ideal_brake = std::max(0.0, -speed_delta);
    const auto accel_limit = std::max(0.0, +accel_);
    const auto brake_limit = std::max(0.0, -accel_);
    input_.parking = false;
    input_.accel = std::min(ideal_accel, accel_limit);
    input_.brake = std::min(ideal_brake, brake_limit);
    input_.steer = steer_;
    return;
  }

  if (gear_ == GearReport::REVERSE) {
    const auto speed_delta = (speed_ - state_.speed) / dt;
    const auto ideal_accel = std::max(0.0, -speed_delta);
    const auto ideal_brake = std::max(0.0, +speed_delta);
    const auto accel_limit = std::max(0.0, +accel_);
    const auto brake_limit = std::max(0.0, -accel_);
    input_.parking = false;
    input_.accel = std::min(ideal_accel, accel_limit) * (-1.0);
    input_.brake = std::min(ideal_brake, brake_limit);
    input_.steer = steer_;
    return;
  }
}

void VehicleController::publish(const rclcpp::Time & stamp)
{
  // Velocity report.
  {
    VelocityReport msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = state_.speed;
    pub_velocity_->publish(msg);
  }

  // Steering report.
  {
    SteeringReport msg;
    msg.stamp = stamp;
    msg.steering_tire_angle = state_.steer;
    pub_steering_->publish(msg);
  }

  // Gear report.
  {
    GearReport msg;
    msg.stamp = stamp;
    msg.report = gear_;
    pub_gear_->publish(msg);
  }
}

}  // namespace autoware_practice_simulator
