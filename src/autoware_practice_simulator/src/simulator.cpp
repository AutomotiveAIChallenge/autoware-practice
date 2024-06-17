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
    kinematics_ = std::make_unique<VehicleKinematics>();
  }

  // Init controller.
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
    controller_ = std::make_unique<VehicleController>(*this, specs, kinematics_.get());
  }

  // Init ROS interface.
  {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pub_markers_ = create_publisher<MarkerArray>("~/markers", rclcpp::QoS(1));
    pub_pose_ = create_publisher<PoseStamped>("~/ground_truth/pose", rclcpp::QoS(1));
    pub_twist_ = create_publisher<TwistStamped>("~/ground_truth/twist", rclcpp::QoS(1));
    pub_path_ = create_publisher<VehiclePath>("~/ground_truth/path", rclcpp::QoS(1));
  }

  // Init simulation settings.
  {
    time_resolution_ = declare_parameter<double>("time_resolution");
    last_time_ = now();
    last_stamp_ = now();
    last_angle_ = 0.0;  // Initial angle is 0.
  }

  // Init simulation timer.
  {
    const auto period = rclcpp::Rate(declare_parameter<double>("rate")).period();
    timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
  }
}

void Simulator::on_timer()
{
  const auto stamp = now();
  execute(stamp);
  publish(stamp);
  controller_->publish(stamp);
}

void Simulator::execute(const rclcpp::Time & stamp)
{
  const auto create_pose_stamped = [](const rclcpp::Time & stamp, const VehicleState & state)
  {
    PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = state.point.x;
    msg.pose.position.y = state.point.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = yaw_to_quaternion(state.angle);
    return msg;
  };

  path_.clear();
  while (last_time_ < stamp) {
    controller_->update(time_resolution_);
    path_.push_back(create_pose_stamped(last_time_, kinematics_->state()));
    last_time_ += rclcpp::Duration::from_seconds(time_resolution_);
  }
}

void Simulator::publish(const rclcpp::Time & stamp)
{
  const auto specs = controller_->specs();
  const auto state = kinematics_->state();
  const auto quaternion = yaw_to_quaternion(state.angle);

  // Vehicle pose transform.
  {
    TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "sim_base_link";
    msg.transform.translation.x = state.point.x;
    msg.transform.translation.y = state.point.y;
    msg.transform.translation.z = 0.0;
    msg.transform.rotation = quaternion;
    tf_broadcaster_->sendTransform(msg);
  }

  // Vehicle pose.
  {
    PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = state.point.x;
    msg.pose.position.y = state.point.y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = quaternion;
    pub_pose_->publish(msg);
  }

  // Vehicle twist.
  {
    double time_diff = (stamp - last_stamp_).seconds();
    double angle_diff = atan2(sin(state.angle - last_angle_), cos(state.angle - last_angle_));
    double angular_velocity_z = angle_diff / time_diff;

    TwistStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "sim_base_link";
    msg.twist.linear.x = state.speed;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = angular_velocity_z;
    pub_twist_->publish(msg);

    // Update last recorded angle and timestamp for next iteration
    last_angle_ = state.angle;
    last_stamp_ = stamp;
  }

  // Vehicle path.
  {
    VehiclePath msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.poses = path_;
    pub_path_->publish(msg);
  }

  // Vehicle markers.
  {
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
    marker.color.r = 0.5;
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
