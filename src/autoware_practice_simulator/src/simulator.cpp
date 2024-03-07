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

#include "simulator.hpp"

namespace autoware_practice_simulator
{

Simulator::Simulator(const rclcpp::NodeOptions & options) : Node("simulator", options)
{
  // Init kinematics.
  {
    VehicleSpecs specs;
    specs.wheelbase = declare_parameter<double>("wheelbase");
    specs.mass = declare_parameter<double>("mass");
    specs.max_speed = declare_parameter<double>("max_speed");
    specs.max_accel = declare_parameter<double>("max_accel");
    specs.max_brake = declare_parameter<double>("max_brake");
    specs.max_steer = declare_parameter<double>("max_steer");
    kinematics_.emplace(specs);
  }

  // Init ROS interface.
  {
    using std::placeholders::_1;

    pub_pose_ = create_publisher<PoseStamped>("~/status/pose", rclcpp::QoS(1));
    pub_velocity_ = create_publisher<VelocityReport>("~/status/velocity", rclcpp::QoS(1));
    pub_steering_ = create_publisher<SteeringReport>("~/status/steering", rclcpp::QoS(1));
    pub_gear_ = create_publisher<GearReport>("~/status/gear", rclcpp::QoS(1));

    sub_control_ = create_subscription<AckermannControlCommand>("~/command/control", rclcpp::QoS(1), std::bind(&Simulator::on_command, this, _1));
    sub_gear_ = create_subscription<GearCommand>("~/command/gear", rclcpp::QoS(1), std::bind(&Simulator::on_gear, this, _1));

    const auto period = rclcpp::Rate(10).period();
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
  }
}

void Simulator::on_command(const AckermannControlCommand & msg)
{
  VehicleInput input;
  input.accel = msg.longitudinal.acceleration;
  input.steer = msg.lateral.steering_tire_angle;
  kinematics_->update_input(input);
}

void Simulator::on_gear(const GearCommand & msg)
{
  if (msg.command == GearCommand::PARK) {
    return kinematics_->update_gear(Gear::Parking);
  }
  if (msg.command == GearCommand::DRIVE) {
    return kinematics_->update_gear(Gear::Drive);
  }
  if (msg.command == GearCommand::REVERSE) {
    return kinematics_->update_gear(Gear::Reverse);
  }
  if (msg.command == GearCommand::NEUTRAL) {
    return kinematics_->update_gear(Gear::Neutral);
  }
}

void Simulator::on_timer()
{
  kinematics_->update_state(0.1);

  const auto stamp = now();

  PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose = kinematics_->pose();
  pub_pose_->publish(pose);

  VelocityReport velocity;
  velocity.header.stamp = stamp;
  velocity.header.frame_id = "base_link";
  velocity.longitudinal_velocity = kinematics_->speed();
  pub_velocity_->publish(velocity);

  SteeringReport steering;
  steering.stamp = stamp;
  steering.steering_tire_angle = kinematics_->steer();
  pub_steering_->publish(steering);

  const auto convert_gear = [](const Gear & gear) {
    switch (gear) {
      case Gear::Parking:
        return GearReport::PARK;
      case Gear::Neutral:
        return GearReport::NEUTRAL;
      case Gear::Drive:
        return GearReport::DRIVE;
      case Gear::Reverse:
        return GearReport::REVERSE;
      default:
        return GearReport::NONE;
    }
  };

  GearReport gear;
  gear.stamp = stamp;
  gear.report = convert_gear(kinematics_->gear());
  pub_gear_->publish(gear);
}

}  // namespace autoware_practice_simulator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_practice_simulator::Simulator)
