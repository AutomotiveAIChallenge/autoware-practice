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

struct VehicleController::VehicleCommand
{
  double accel;
  double brake;
  double steer;
};

VehicleController::VehicleController(rclcpp::Node & node, const VehicleSpecs & specs, VehicleKinematics * kinematics)
{
  using std::placeholders::_1;
  sub_control_ = node.create_subscription<ControlCommand>("~/command/control", rclcpp::QoS(1), std::bind(&VehicleController::on_command, this, _1));
  sub_gear_ = node.create_subscription<GearCommand>("~/command/gear", rclcpp::QoS(1), std::bind(&VehicleController::on_gear, this, _1));
  pub_gear_ = node.create_publisher<GearReport>("~/status/gear", rclcpp::QoS(1));
  pub_velocity_ = node.create_publisher<VelocityReport>("~/status/velocity", rclcpp::QoS(1));
  pub_steering_ = node.create_publisher<SteeringReport>("~/status/steering", rclcpp::QoS(1));

  kinematics_ = kinematics;
  specs_ = specs;
  gear_ = GearReport::DRIVE;
  actual_speed_ = 0.0;
  actual_accel_ = 0.0;
  actual_steer_ = 0.0;
  target_accel_ = 0.0;
  target_steer_ = 0.0;
}

void VehicleController::on_command(const ControlCommand & msg)
{
  target_accel_ = msg.longitudinal.acceleration;
  target_steer_ = msg.lateral.steering_tire_angle;
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

void VehicleController::update(double dt)
{
  const auto current = kinematics_->state();
  const auto command = create_command();

  const auto brake_sign = std::signbit(current.speed) ? +1.0 : -1.0;
  const auto brake_limit = std::min(specs_.max_brake, std::abs(current.speed) / dt);
  const auto brake = std::clamp(command.brake, 0.0, brake_limit) * brake_sign;
  const auto accel = std::clamp(command.accel, -specs_.max_accel, +specs_.max_accel);
  const auto steer = std::clamp(command.steer, -specs_.max_steer, +specs_.max_steer);

  actual_accel_ = accel + brake;
  actual_steer_ = steer;
  actual_speed_ = actual_speed_ + (actual_accel_ * dt);
  actual_speed_ = std::clamp(actual_speed_, -specs_.max_speed, +specs_.max_speed);

  constexpr double steer_eps = 1e-3;
  const double radius = (std::abs(actual_steer_) < steer_eps) ? 0.0 : specs_.wheel_base / std::tan(actual_steer_);
  heading_rate_ = (radius == 0.0) ? 0.0 : actual_speed_ / radius;
  kinematics_->update(dt, ArcPath{radius, actual_speed_});
}

VehicleController::VehicleCommand VehicleController::create_command()
{
  if (gear_ == GearReport::DRIVE) {
    const auto accel = std::max(0.0, +target_accel_);
    const auto brake = std::max(0.0, -target_accel_);
    const auto steer = target_steer_;
    return VehicleCommand{accel, brake, steer};
  }

  if (gear_ == GearReport::REVERSE) {
    const auto accel = std::max(0.0, -target_accel_);
    const auto brake = std::max(0.0, +target_accel_);
    const auto steer = target_steer_;
    return VehicleCommand{-1.0 * accel, brake, steer};
  }

  if (gear_ == GearReport::PARK) {
    const auto accel = 0.0;
    const auto brake = specs_.max_brake;
    const auto steer = target_steer_;
    return VehicleCommand{accel, brake, steer};
  }

  if (gear_ == GearReport::NEUTRAL) {
    const auto accel = 0.0;
    const auto brake = 0.0;
    const auto steer = target_steer_;
    return VehicleCommand{accel, brake, steer};
  }

  const auto accel = 0.0;
  const auto brake = specs_.max_brake;
  const auto steer = target_steer_;
  return VehicleCommand{accel, brake, steer};
}

void VehicleController::publish(const rclcpp::Time & stamp)
{
  // Velocity report.
  {
    VelocityReport msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.longitudinal_velocity = actual_speed_;
    msg.lateral_velocity = 0.0;  
    msg.heading_rate = heading_rate_;
    pub_velocity_->publish(msg);
  }

  // Steering report.
  {
    SteeringReport msg;
    msg.stamp = stamp;
    msg.steering_tire_angle = actual_steer_;
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
