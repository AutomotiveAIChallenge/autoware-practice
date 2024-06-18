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

#include "backward.hpp"

#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("backward")
{
  gear_.report = GearReport::NONE;

  using std::placeholders::_1;
  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  pub_gear_ = create_publisher<GearCommand>("/control/command/gear_cmd", rclcpp::QoS(1));
  sub_gear_ = create_subscription<GearReport>(
    "/vehicle/status/gear_status", rclcpp::QoS(1), std::bind(&SampleNode::on_gear, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::on_gear(const GearReport & msg)
{
  gear_ = msg;
}

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;
  if (gear_.report == GearReport::REVERSE) {
    command.longitudinal.speed = 0.0;
    command.longitudinal.acceleration = -2.5;
  } else {
    command.longitudinal.speed = 0.0;
    command.longitudinal.acceleration = -2.5;
  }
  command.lateral.steering_tire_angle = 0.0;
  pub_command_->publish(command);

  GearCommand gear;
  gear.stamp = stamp;
  gear.command = GearCommand::REVERSE;
  pub_gear_->publish(gear);
}

}  // namespace autoware_practice_course

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<autoware_practice_course::SampleNode>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
