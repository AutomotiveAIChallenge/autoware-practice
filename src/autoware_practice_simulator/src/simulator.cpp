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

#include "simulator.hpp"

#include <memory>
#include <vector>

namespace autoware_practice_simulator
{

Simulator::Simulator(const rclcpp::NodeOptions & options) : Node("simulator", options)
{
  // Init kinematics.
  {
    VehicleSpecs specs;
    const auto overhang = declare_parameter<std::vector<double>>("overhang");
    if (overhang.size() != specs.overhang.size()) {
      throw std::invalid_argument("overhang size should be " + std::to_string(specs.overhang.size()));
    }
    std::copy_n(overhang.begin(), specs.overhang.size(), specs.overhang.begin());
    specs.height = declare_parameter<double>("height");
    specs.wheel_tread = declare_parameter<double>("wheel_tread");
    specs.wheel_base = declare_parameter<double>("wheel_base");
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
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_control_ = create_subscription<AckermannControlCommand>("~/command/control", rclcpp::QoS(1), std::bind(&Simulator::on_command, this, _1));
    sub_gear_ = create_subscription<GearCommand>("~/command/gear", rclcpp::QoS(1), std::bind(&Simulator::on_gear, this, _1));

    pub_pose_ = create_publisher<PoseStamped>("~/status/pose", rclcpp::QoS(1));
    pub_velocity_ = create_publisher<VelocityReport>("~/status/velocity", rclcpp::QoS(1));
    pub_steering_ = create_publisher<SteeringReport>("~/status/steering", rclcpp::QoS(1));
    pub_gear_ = create_publisher<GearReport>("~/status/gear", rclcpp::QoS(1));
    pub_markers_ = create_publisher<MarkerArray>("~/markers", rclcpp::QoS(1));

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
  const auto stamp = now();
  kinematics_->update_state(0.1);

  {
    const auto pose = kinematics_->pose();
    TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id = "sim_base_link";
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  // Vehicle pose.
  {
    PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = "map";
    pose.pose = kinematics_->pose();
    pub_pose_->publish(pose);
  }

  // Velocity report.
  {
    VelocityReport velocity;
    velocity.header.stamp = stamp;
    velocity.header.frame_id = "base_link";
    velocity.longitudinal_velocity = kinematics_->speed();
    pub_velocity_->publish(velocity);
  }

  // Steering report.
  {
    SteeringReport steering;
    steering.stamp = stamp;
    steering.steering_tire_angle = kinematics_->steer();
    pub_steering_->publish(steering);
  }

  // Gear report.
  {
    const auto convert_gear = [](const Gear & gear)
    {
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

  // Vehicle markers.
  {
    const auto specs = kinematics_->specs();
    const auto base_x = specs.overhang[0] + specs.wheel_base;
    const auto full_x = specs.overhang[0] + specs.overhang[1] + specs.wheel_base;
    const auto full_y = specs.overhang[2] + specs.overhang[3] + specs.wheel_tread;

    Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = "sim_base_link";
    marker.ns = "vehicle";
    marker.id = 0;
    marker.action = Marker::ADD;
    marker.type = Marker::CUBE;
    marker.pose.position.x = base_x / 2.0;
    marker.pose.position.z = specs.height / 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = full_x;
    marker.scale.y = full_y;
    marker.scale.z = specs.height;
    marker.color.a = 1.0;
    marker.color.r = 0.7;
    marker.color.g = 0.7;
    marker.color.b = 0.7;

    MarkerArray markers;
    markers.markers.push_back(marker);
    pub_markers_->publish(markers);
  }
}

}  // namespace autoware_practice_simulator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_practice_simulator::Simulator)
