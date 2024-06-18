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

#include "longitudinal_controller.hpp"

#include <limits>
#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("longitudinal_controller"), kp_(0.0)
{
  using std::placeholders::_1;
  declare_parameter<double>("kp", kp_);
  get_parameter("kp", kp_);

  pub_command_ = create_publisher<AckermannControlCommand>("/control/command/control_cmd", rclcpp::QoS(1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1), std::bind(&SampleNode::update_target_velocity, this, _1));
  sub_kinematic_state_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_target_velocity(const Trajectory & msg)
{
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_waypoint_index = 0;

  for (size_t i = 0; i < msg.points.size(); ++i) {
    double dx = msg.points[i].pose.position.x - current_pose_.x;
    double dy = msg.points[i].pose.position.y - current_pose_.y;
    double dz = msg.points[i].pose.position.z - current_pose_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < min_distance) {
      min_distance = distance;
      closest_waypoint_index = i;
    }
  }

  target_velocity_ = msg.points[closest_waypoint_index].longitudinal_velocity_mps;
};

void SampleNode::update_current_state(const Odometry & msg)
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_pose_ = msg.pose.pose.position;  // 現在の車両の位置を更新する
};

void SampleNode::on_timer()
{
  const auto stamp = now();

  AckermannControlCommand command;
  command.stamp = stamp;

  double velocity_error = target_velocity_ - current_velocity_;
  command.longitudinal.acceleration = kp_ * velocity_error;
  command.longitudinal.speed = target_velocity_;  // メッセージ型としてはspeedがあるが、vehiclle
                                                  // interface側では加速度しか受け取っていない。

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
