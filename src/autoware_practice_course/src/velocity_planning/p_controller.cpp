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

#include "p_controller.hpp"

#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("p_controller")
{
  declare_parameter<double>("kp", 0.0);
  declare_parameter<double>("target_velocity", 1.0);

  get_parameter("kp", kp_);
  get_parameter("target_velocity", target_velocity_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));

  velocity_subscriber_ = create_subscription<VelocityReport>(
    "/vehicle/status/velocity_status", 10,
    [this](const VelocityReport::SharedPtr msg) { current_velocity_ = msg->longitudinal_velocity; });

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;

  double velocity_error = target_velocity_ - current_velocity_;
  command.longitudinal.acceleration = kp_ * velocity_error;
  command.longitudinal.speed = target_velocity_;

  command.lateral.steering_tire_angle = 0.0;

  pub_command_->publish(command);
}

}  // namespace autoware_practice_course

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_course::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
