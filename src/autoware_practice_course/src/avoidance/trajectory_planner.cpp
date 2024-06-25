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

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("trajectory_planner")
{
  using std::placeholders::_1;
  GRID_RESOLUTION_ = 1.0;  // 1セルのサイズ（メートル）
  state_num_ = 10;

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

std::vector<SampleNode::Trajectory> SampleNode::create_trajectory_set()
{
  // 目標状態集合を計算
  std::vector<TrajectoryPoint> target_trajectory_point_set = create_target_state_set();

  std::vector<Trajectory> trajectory_set;
  Eigen::Quaterniond current_q;
  Eigen::Quaterniond target_q;

  current_q.x() = current_orientation_.x;
  current_q.y() = current_orientation_.y;
  current_q.z() = current_orientation_.z;
  current_q.w() = current_orientation_.w;

  target_q.x() = target_trajectory_point_set[1].pose.orientation.x;
  target_q.y() = target_trajectory_point_set[1].pose.orientation.y;
  target_q.z() = target_trajectory_point_set[1].pose.orientation.z;
  target_q.w() = target_trajectory_point_set[1].pose.orientation.w;

  double current_inclination = quaternionToInclination(current_q);
  double target_inclination = quaternionToInclination(target_q);

  // 車両の位置姿勢と目標状態集合をエルミート補間し、軌道を生成
  for (const auto & target_trajectory_point : target_trajectory_point_set) {
    double num_points = 10;
    // 車両の位置姿勢と目標状態をエルミート補間
    std::vector<Point> interpolated_points = hermiteInterpolate(
      current_position_, target_trajectory_point.pose.position, current_inclination, target_inclination, num_points);
    Trajectory trajectory_candidate;

    // 補間された曲線状のstd::vector<Point>をTrajectoryに変換
    for (const auto & interpolated_point : interpolated_points) {
      TrajectoryPoint trajectory_point;
      trajectory_point.pose.position = interpolated_point;
      trajectory_candidate.points.push_back(trajectory_point);
    }

    // 補間された曲線状のTrajectory Pointを生成
    trajectory_set.push_back(trajectory_candidate);
  }

  return trajectory_set;
}

// 目標状態集合を計算
std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> SampleNode::create_target_state_set()
{
  // 車両に最も近いtrajectory pointを取得
  double min_distance = std::numeric_limits<double>::max();
  size_t closest_waypoint_index = 0;

  for (size_t i = 0; i < reference_trajectory_.points.size(); ++i) {
    double dx = reference_trajectory_.points[i].pose.position.x - current_position_.x;
    double dy = reference_trajectory_.points[i].pose.position.y - current_position_.y;
    double dz = reference_trajectory_.points[i].pose.position.z - current_position_.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < min_distance) {
      min_distance = distance;
      closest_waypoint_index = i;
    }
  }
  int target_trajectory_point_index = closest_waypoint_index + 10;
  // 10個先のtrajectory pointを取得
  TrajectoryPoint target_trajectory_point = reference_trajectory_.points[target_trajectory_point_index];

  Eigen::Quaterniond q = vectorToQuaternion(
    pointToVector3d(reference_trajectory_.points[target_trajectory_point_index].pose.position),
    pointToVector3d(reference_trajectory_.points[target_trajectory_point_index + 1].pose.position));
  target_trajectory_point.pose.orientation.x = q.x();
  target_trajectory_point.pose.orientation.y = q.y();
  target_trajectory_point.pose.orientation.z = q.z();
  target_trajectory_point.pose.orientation.w = q.w();

  // 目標状態集合を生成
  // target_trajectory_pointの姿勢に直交する方向に並ぶstate_num個の状態を生成
  // クォータニオンを回転行列に変換
  Eigen::Matrix3d R = q.toRotationMatrix();

  // 直交するベクトルの選択（第二列ベクトル）
  Eigen::Vector3d v = R.col(1);
  // 点の配置
  double d = 1.0;  // 点と点の間の距離
  std::vector<TrajectoryPoint> target_state_set;
  for (int n = -4; n <= 5; ++n) {
    TrajectoryPoint target_state = target_trajectory_point;
    target_state.pose.position = target_trajectory_point.pose.position + vector3dToPoint(n * d * v);
    target_state_set.push_back(target_state);
  }

  return target_state_set;
}

// + 演算子のオーバーロード
geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  geometry_msgs::msg::Point result;
  result.x = p1.x + p2.x;
  result.y = p1.y + p2.y;
  result.z = p1.z + p2.z;
  return result;
}

Eigen::Vector3d SampleNode::pointToVector3d(const geometry_msgs::msg::Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point SampleNode::vector3dToPoint(const Eigen::Vector3d & vector)
{
  geometry_msgs::msg::Point point;
  point.x = vector.x();
  point.y = vector.y();
  point.z = vector.z();
  return point;
}

Eigen::Quaterniond SampleNode::vectorToQuaternion(const Eigen::Vector3d & start, const Eigen::Vector3d & end)
{
  // 方向ベクトルの計算
  Eigen::Vector3d direction = (end - start).normalized();

  // 基準ベクトル（例：x軸）
  Eigen::Vector3d referenceVector(1.0, 0.0, 0.0);

  // 回転軸の計算
  Eigen::Vector3d axis = referenceVector.cross(direction);
  axis.normalize();

  // 回転角度の計算
  double angle = acos(referenceVector.dot(direction));

  // 回転をクォータニオンに変換
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
  return q;
}

std::vector<std::vector<float>> SampleNode::create_costmap()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  // 変換関数を呼び出し
  pcl::fromROSMsg(pointcloud_, *pointcloud_pcl);
  // pointcloud_を元にcostmapを生成
  // グリッドマップの初期化
  const double GRID_WIDTH = 100.0;
  const double GRID_HEIGHT = 100.0;

  std::vector<std::vector<float>> costmap(GRID_WIDTH, std::vector<float>(GRID_HEIGHT, 0.0));

  // 点群をグリッドマップに変換
  for (const auto & point : pointcloud_pcl->points) {
    int x_index = static_cast<int>(point.x / GRID_RESOLUTION_);
    int y_index = static_cast<int>(point.y / GRID_RESOLUTION_);

    if (x_index >= 0 && x_index < GRID_WIDTH && y_index >= 0 && y_index < GRID_HEIGHT) {
      costmap[x_index][y_index] = 100.0;  // 点が存在する格子は評価関数を高く設定
    }
  }
  // 周囲の格子に評価関数を設定
  const int KERNEL_SIZE = 1;           // 評価関数を設定する範囲（カーネルサイズ）
  const float SURROUNDING_COST = 0.5;  // 周囲の格子の評価関数

  for (int x = 0; x < GRID_WIDTH; ++x) {
    for (int y = 0; y < GRID_HEIGHT; ++y) {
      if (costmap[x][y] == 1.0) {
        for (int dx = -KERNEL_SIZE; dx <= KERNEL_SIZE; ++dx) {
          for (int dy = -KERNEL_SIZE; dy <= KERNEL_SIZE; ++dy) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < GRID_WIDTH && ny >= 0 && ny < GRID_HEIGHT) {
              costmap[nx][ny] += SURROUNDING_COST;
            }
          }
        }
      }
    }
  }

  return costmap;
}

SampleNode::Trajectory SampleNode::evaluate_trajectory(
  const std::vector<SampleNode::Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap)
{
  Trajectory best_trajectory;
  std::vector<float> trajectory_cost(trajectory_set.size(), 0.0f);
  int index = 0;

  // trajectory_setをcostmapで評価し、最適な軌道を選択
  for (const auto & trajectory_candidate : trajectory_set) {
    // trajectoryをcostmapで評価
    for (const auto & trajectory_point : trajectory_candidate.points) {
      int x_index = static_cast<int>(trajectory_point.pose.position.x / GRID_RESOLUTION_);
      int y_index = static_cast<int>(trajectory_point.pose.position.y / GRID_RESOLUTION_);
      trajectory_cost[index] += costmap[x_index][y_index];
    }
    index++;
    // 評価値を計算
  }

  // 最小コストのインデックスを出力
  int min_index =
    std::distance(trajectory_cost.begin(), std::min_element(trajectory_cost.begin(), trajectory_cost.end()));
  best_trajectory = trajectory_set[min_index];
  return best_trajectory;
}

double SampleNode::quaternionToInclination(Eigen::Quaterniond q)
{
  double inclination = 2.0 * std::atan2(q.z(), q.w());
  return inclination;
}

// エルミート補間関数
std::vector<geometry_msgs::msg::Point> SampleNode::hermiteInterpolate(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1, double m0, double m1, int numPoints)
{
  std::vector<geometry_msgs::msg::Point> interpolatedPoints;

  Eigen::Vector3d v0 = pointToVector3d(p0);
  Eigen::Vector3d v1 = pointToVector3d(p1);

  for (int i = 0; i < numPoints; ++i) {
    double t = static_cast<double>(i) / (numPoints - 1);
    double t2 = t * t;
    double t3 = t2 * t;

    double h00 = 2 * t3 - 3 * t2 + 1;
    double h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2;
    double h11 = t3 - t2;

    Eigen::Vector3d interpolatedVector = h00 * v0 + h10 * m0 * (v1 - v0) + h01 * v1 + h11 * m1 * (v1 - v0);

    geometry_msgs::msg::Point point;
    point.x = interpolatedVector.x();
    point.y = interpolatedVector.y();
    point.z = interpolatedVector.z();

    interpolatedPoints.push_back(point);
  }

  return interpolatedPoints;
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
