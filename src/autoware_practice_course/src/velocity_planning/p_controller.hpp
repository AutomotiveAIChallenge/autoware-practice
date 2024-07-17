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

#ifndef VELOCITY_PLANNING__P_CONTROLLER_HPP_
#define VELOCITY_PLANNING__P_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>

namespace autoware_practice_course
{

class PControllerNode : public rclcpp::Node
{
public:
  PControllerNode();

private:
  using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;

  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_command_;

  rclcpp::Subscription<VelocityReport>::SharedPtr velocity_subscriber_;

  double current_velocity_;

  double target_velocity_;

  double kp_;

  void velocity_callback(const VelocityReport::SharedPtr msg);
};

}  // namespace autoware_practice_course

#endif  // VELOCITY_PLANNING__P_CONTROLLER_HPP_
