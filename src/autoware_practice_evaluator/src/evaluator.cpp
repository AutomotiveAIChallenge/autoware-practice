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

#include "evaluator.hpp"

#include <string>

namespace autoware_practice_evaluator
{

Evaluator::Evaluator(const rclcpp::NodeOptions & options) : Node("evaluator", options)
{
  using std::placeholders::_1;
  sub_pose_ = create_subscription<PoseStamped>("~/pose", rclcpp::QoS(1), std::bind(&Evaluator::on_pose, this, _1));
  sub_path_ = create_subscription<VehiclePath>("~/path", rclcpp::QoS(1), std::bind(&Evaluator::on_path, this, _1));
  pub_result_ = create_publisher<JudgeStatus>("~/result", rclcpp::QoS(1));
  pub_markers_ = create_publisher<MarkerArray>("~/markers", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(10.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });

  const auto file = declare_parameter<std::string>("judge_file");
  condition_tree_ = Condition::make_unique(YAML::LoadFile(file));
  condition_list_ = Condition::descendants(condition_tree_.get());
}

void Evaluator::on_pose(const PoseStamped & msg)
{
  data_.point.x = msg.pose.position.x;
  data_.point.y = msg.pose.position.y;
  data_.point.z = msg.pose.position.z;
}

void Evaluator::on_path(const VehiclePath & msg)
{
  data_.path.clear();
  for (const auto & pose : msg.poses) {
    Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = pose.pose.position.z;
    data_.path.push_back(point);
  }
}

void Evaluator::on_timer()
{
  const auto convert = [](TriState state)
  {
    // clang-format off
    switch(state) {
      case TriState::Failure: return JudgeStatus::FAILURE;
      case TriState::Judging: return JudgeStatus::JUDGING;
      case TriState::Success: return JudgeStatus::SUCCESS;
      default:                return JudgeStatus::INVALID;
    }
    // clang-format on
  };

  const auto stamp = now();

  // Publish test result.
  {
    JudgeStatus msg;
    msg.result = convert(condition_tree_->update(data_));
    pub_result_->publish(msg);
  }

  // Publish markers.
  {
    int32_t index = 0;
    MarkerArray msg;
    for (const auto & condition : condition_list_) {
      for (const auto & marker : condition->visualize()) {
        msg.markers.push_back(marker.message());
        msg.markers.back().id = index++;
        msg.markers.back().header.stamp = stamp;
      }
    }
    pub_markers_->publish(msg);
  }
}

}  // namespace autoware_practice_evaluator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_practice_evaluator::Evaluator)
