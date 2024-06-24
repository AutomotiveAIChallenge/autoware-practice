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

#include "trajectory_planner.hpp"

#include <cmath>
#include <memory>

namespace autoware_practice_course
{
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
SampleNode::SampleNode() : Node("trajectory_planner")
{
  using std::placeholders::_1;

  pub_trajectory_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1));
  sub_kinematic_state_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS(1),
    std::bind(&SampleNode::update_reference_trajectory, this, _1));
  sub_pointcloud_ = create_subscription<PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", rclcpp::QoS(1),
    std::bind(&SampleNode::update_pointcloud, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_current_state(const Odometry & msg)  // called by sub_kinematic_state_
{
  current_velocity_ = msg.twist.twist.linear.x;
  current_position_ = msg.pose.pose.position;
  current_orientation_ = msg.pose.pose.orientation;
};

void SampleNode::update_reference_trajectory(const Trajectory & msg)  // called by sub_trajectory_
{
  reference_trajectory_ = msg;
};

void SampleNode::update_pointcloud(const PointCloud2 & msg)  // called by sub_pointcloud_
{
  pointcloud_ = msg;
};

void SampleNode::on_timer()
{
  Trajectory trajectory = create_trajectory();
  pub_trajectory_->publish(trajectory);
}

SampleNode::Trajectory SampleNode::create_trajectory()  // called by on_timer()
{
  // state lattice planner
  // create trajectory library
  std::vector<Trajectory> trajectory_set = create_trajectory_set();

  // create costmap
  std::vector<std::vector<float>> costmap = create_costmap();

  // evaluate trahectories by the cost map
  Trajectory best_trajectory = evaluate_trajectory(trajectory_set, costmap);

  return best_trajectory;
}

std::vector<Trajectory> SampleNode::create_trajectory_set()
{
  std::vector<Trajectory> trajectory_set;
  int state_num = 10;
  // 目標trajectory pointを取得
  TrajectoryPoint target_trajectory_point = calculate_target_trajectory_point();

  // 目標trajectory pointから目標状態集合を計算
  std::vector<TrajectoryPoint> traget_trajectory_point_set =
    create_target_state_set(target_trajectory_point, state_num);

  // 車両の位置姿勢と目標状態集合をエルミート補間し、軌道を生成
  for (const auto & traget_trajectory_point : traget_trajectory_point_set) {
    // 車両の位置姿勢と目標状態をエルミート補間

    // 補間された曲線状のTrajectory Pointを生成
  }

  return trajectory_set;
}

SampleNode::TrajectoryPoint SampleNode::calculate_target_trajectory_point()
{
  TrajectoryPoint target_trajectory_point;

  // 車両に最も近いtrajectory pointを取得
  TrajectoryPoint nearest_trajectory_point = calculate_nearest_trajectory_point();

  // 10個先のtrajectory pointを取得

  return target_trajectory_point;
}

std::vector<std::vector<float>> SampleNode::create_costmap()
{
  std::vector<std::vector<float>> costmap;
  // pointcloud_を元にcostmapを生成

  return costmap;
}

SampleNode::Trajectory SampleNode::evaluate_trajectory(
  const std::vector<SampleNode::Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap)
{
  Trajectory best_trajectory;
  // trajectory_setをcostmapで評価し、最適な軌道を選択
  for (const auto & trajectory : trajectory_set) {
    // trajectoryをcostmapで評価

    // 評価値を計算

    // 評価値が最小の場合、best_trajectoryを更新
  }
  return best_trajectory;
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
