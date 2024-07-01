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
  GRID_RESOLUTION_ = 1;  // 1セルのサイズ（メートル）
  GRID_WIDTH_ = 100.0;
  GRID_HEIGHT_ = 100.0;
  state_num_ = 10;

  pub_trajectory_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1));
  pub_trajectory_candidate_ =
    create_publisher<Trajectory>("/planning/scenario_planning/trajectory_candidate", rclcpp::QoS(1));
  pub_costmap_ =
    create_publisher<autoware_practice_msgs::msg::FloatGrid>("/planning/scenario_planning/costmap", rclcpp::QoS(1));
  sub_kinematic_state_ = create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_current_state, this, _1));
  sub_trajectory_ = create_subscription<Trajectory>(
    "/planning/trajectory_loader/trajectory", rclcpp::QoS(1),
    std::bind(&SampleNode::update_reference_trajectory, this, _1));
  sub_pointcloud_ = create_subscription<PointCloud2>(
    "/perception/obstacle_segmentation/pointcloud", rclcpp::QoS(1),
    std::bind(&SampleNode::update_pointcloud, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_current_state(const Odometry & msg)  // called by sub_kinematic_state_
{
  // RCLCPP_INFO(this->get_logger(), "Updating current state.");
  current_velocity_ = msg.twist.twist.linear.x;
  current_position_ = msg.pose.pose.position;
  current_orientation_ = msg.pose.pose.orientation;
  current_state_initialized_ = true;
};

void SampleNode::update_reference_trajectory(const Trajectory & msg)  // called by sub_trajectory_
{
  // RCLCPP_INFO(this->get_logger(), "Updating reference trajectory.");
  reference_trajectory_ = msg;
  reference_trajectory_initialized_ = true;
};

void SampleNode::update_pointcloud(const PointCloud2 & msg)  // called by sub_pointcloud_
{
  pointcloud_ = msg;
  pointcloud_initialized_ = true;
  // RCLCPP_INFO(this->get_logger(), "PointCloud received and updated.");
};

void SampleNode::on_timer()
{
  if (!current_state_initialized_ || !reference_trajectory_initialized_ || !pointcloud_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for all initial data to be received.");
  } else {
    // RCLCPP_INFO(this->get_logger(), "Timer triggered, creating trajectory.");
    create_trajectory();
    pub_trajectory_->publish(best_trajectory_);
    pub_trajectory_candidate_->publish(trajectory_candidate_);
    // RCLCPP_INFO(this->get_logger(), "Costmap size: %zu",costmap_.size());
    /*
    for (size_t i = 0; i < costmap_.size(); ++i) {
        std::string row = "Row " + std::to_string(i) + ": ";
        for (size_t j = 0; j < costmap_[i].size(); ++j) {
            row += std::to_string(costmap_[i][j]) + " ";
        }
        RCLCPP_INFO(this->get_logger(), row.c_str());
    }
    */
    auto costmap_msg = autoware_practice_msgs::msg::FloatGrid();
    costmap_msg.width = GRID_WIDTH_;
    costmap_msg.height = GRID_HEIGHT_;

    // RCLCPP_INFO(this->get_logger(), "Costmap.data[9999]: %f", costmap_[99][99]);

    // 2次元配列を1次元配列に変換
    for (const auto & row : costmap_) {
      costmap_msg.data.insert(costmap_msg.data.end(), row.begin(), row.end());
    }

    pub_costmap_->publish(costmap_msg);
  }
}

void SampleNode::create_trajectory()  // called by on_timer()
{
  // RCLCPP_INFO(this->get_logger(), "Creating trajectory.");
  //  state lattice planner
  //  create trajectory library
  std::vector<Trajectory> trajectory_set = create_trajectory_set();

  // create costmap
  costmap_ = create_costmap();

  // evaluate trahectories by the cost map
  best_trajectory_ = evaluate_trajectory(trajectory_set, costmap_);
  // RCLCPP_INFO(this->get_logger(), "Logging trajectory with %zu points", best_trajectory.points.size());
  /*
  for (size_t i = 0; i < best_trajectory.points.size(); ++i) {
    const auto & point = best_trajectory.points[i];
    RCLCPP_INFO(
      this->get_logger(), "Point %zu: x=%f, y=%f, z=%f", i, point.pose.position.x, point.pose.position.y,
      point.pose.position.z);

  }
  */
}

std::vector<SampleNode::Trajectory> SampleNode::create_trajectory_set()
{
  // RCLCPP_INFO(this->get_logger(), "Creating trajectory set.");

  // 目標状態集合を計算
  std::vector<TrajectoryPoint> target_trajectory_point_set = create_target_state_set();
  // RCLCPP_INFO(this->get_logger(), "Target trajectory point set created. Size: %zu",
  // target_trajectory_point_set.size());

  if (target_trajectory_point_set.size() < 2) {
    RCLCPP_ERROR(this->get_logger(), "Target trajectory point set size is less than 2.");
    return {};
  }

  std::vector<Trajectory> trajectory_set;
  Eigen::Quaterniond current_q;
  Eigen::Quaterniond target_q;

  current_q.x() = current_orientation_.x;
  current_q.y() = current_orientation_.y;
  current_q.z() = current_orientation_.z;
  current_q.w() = current_orientation_.w;
  /*RCLCPP_INFO(
    this->get_logger(), "Current orientation set: x=%f, y=%f, z=%f, w=%f", current_orientation_.x,
    current_orientation_.y, current_orientation_.z, current_orientation_.w);
  */
  target_q.x() = target_trajectory_point_set[1].pose.orientation.x;
  target_q.y() = target_trajectory_point_set[1].pose.orientation.y;
  target_q.z() = target_trajectory_point_set[1].pose.orientation.z;
  target_q.w() = target_trajectory_point_set[1].pose.orientation.w;
  /*RCLCPP_INFO(
    this->get_logger(), "Target orientation set: x=%f, y=%f, z=%f, w=%f", target_q.x(), target_q.y(), target_q.z(),
    target_q.w());
  */
  // double current_inclination = quaternionToInclination(current_q);
  // double target_inclination = quaternionToInclination(target_q);
  Eigen::Vector3d current_vector = quaternionToVector(current_q);
  Eigen::Vector3d target_vector = quaternionToVector(target_q);
  /*RCLCPP_INFO(
    this->get_logger(), "Current inclination: %f, Target inclination: %f", current_inclination, target_inclination);
  */

  // 車両の位置姿勢と目標状態集合をエルミート補間し、軌道を生成
  Trajectory trajectory_candidate;
  for (const auto & target_trajectory_point : target_trajectory_point_set) {
    /*
    RCLCPP_INFO(
      this->get_logger(), "Processing target trajectory point at position: (%f, %f, %f)",
      target_trajectory_point.pose.position.x, target_trajectory_point.pose.position.y,
      target_trajectory_point.pose.position.z);
    */

    double num_points = 20;

    // 車両の位置姿勢と目標状態をエルミート補間
    std::vector<Point> interpolated_points = hermiteInterpolate(
      current_position_, target_trajectory_point.pose.position, current_vector, target_vector, num_points);

    Trajectory trajectorys;

    // 補間された曲線状のstd::vector<Point>をTrajectoryに変換
    for (const auto & interpolated_point : interpolated_points) {
      TrajectoryPoint trajectory_point;
      trajectory_point.pose.position = interpolated_point;
      trajectory_point.longitudinal_velocity_mps = 2.0;
      trajectorys.points.push_back(trajectory_point);
      trajectory_candidate.points.push_back(trajectory_point);
      /*
            RCLCPP_INFO(
              this->get_logger(), "Added interpolated point to trajectory candidate. Point position: (%f, %f, %f)",
              interpolated_point.x, interpolated_point.y, interpolated_point.z);
      */
    }

    /*
        RCLCPP_INFO(
          this->get_logger(), "Trajectory candidate created. Number of points: %zu",
       trajectory_candidate.points.size());
    */
    // 補間された曲線状のTrajectory Pointを生成
    trajectory_set.push_back(trajectorys);
    /*
        RCLCPP_INFO(
          this->get_logger(), "Added trajectory candidate to trajectory set. Current trajectory set size: %zu",
          trajectory_set.size());
    */
  }
  trajectory_candidate_ = trajectory_candidate;

  // RCLCPP_INFO(this->get_logger(), "Trajectory set created. Size: %zu", trajectory_set.size());

  return trajectory_set;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> SampleNode::create_target_state_set()
{
  // RCLCPP_INFO(this->get_logger(), "Creating target state set.");

  if (reference_trajectory_.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Reference trajectory is empty.");
    return {};  // 空のvectorを返す
  } else {
    // 車両に最も近いtrajectory pointを取得
    double min_distance = std::numeric_limits<double>::max();
    size_t closest_waypoint_index = 0;
    /*
    RCLCPP_INFO(
      this->get_logger(), "Creating target state set. Number of points in reference trajectory: %zu",
      reference_trajectory_.points.size());
    */

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

    // RCLCPP_INFO(this->get_logger(), "Closest waypoint index: %zu, Distance: %f", closest_waypoint_index,
    // min_distance);

    int target_trajectory_point_index = closest_waypoint_index + 10;
    // RCLCPP_INFO(this->get_logger(), "Target trajectory point index: %d", target_trajectory_point_index);

    // 10個先のtrajectory pointを取得
    autoware_auto_planning_msgs::msg::TrajectoryPoint target_trajectory_point =
      reference_trajectory_.points[target_trajectory_point_index];

    Eigen::Quaterniond q = vectorToQuaternion(
      pointToVector3d(reference_trajectory_.points[target_trajectory_point_index].pose.position),
      pointToVector3d(reference_trajectory_.points[target_trajectory_point_index + 1].pose.position));
    target_trajectory_point.pose.orientation.x = q.x();
    target_trajectory_point.pose.orientation.y = q.y();
    target_trajectory_point.pose.orientation.z = q.z();
    target_trajectory_point.pose.orientation.w = q.w();
    /*
        RCLCPP_INFO(
          this->get_logger(), "Target trajectory point position: (%f, %f, %f)", target_trajectory_point.pose.position.x,
          target_trajectory_point.pose.position.y, target_trajectory_point.pose.position.z);
    */
    // 目標状態集合を生成
    // target_trajectory_pointの姿勢に直交する方向に並ぶstate_num個の状態を生成
    // クォータニオンを回転行列に変換
    Eigen::Matrix3d R = q.toRotationMatrix();

    // 直交するベクトルの選択（第二列ベクトル）
    Eigen::Vector3d v = R.col(1);

    // RCLCPP_INFO(this->get_logger(), "Orthogonal vector: (%f, %f, %f)", v.x(), v.y(), v.z());

    // 点の配置
    double d = 1.0;  // 点と点の間の距離
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> target_state_set;
    for (int n = -4; n <= 4; ++n) {
      autoware_auto_planning_msgs::msg::TrajectoryPoint target_state = target_trajectory_point;
      target_state.pose.position.x += n * d * v.x();
      target_state.pose.position.y += n * d * v.y();
      target_state.pose.position.z += n * d * v.z();
      target_state_set.push_back(target_state);
      /*
            RCLCPP_INFO(
              this->get_logger(), "Added target state: index %d, position (%f, %f, %f)", n,
         target_state.pose.position.x, target_state.pose.position.y, target_state.pose.position.z);
              */
    }

    return target_state_set;
  }
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
  // RCLCPP_INFO(this->get_logger(), "Creating costmap.");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
  // 変換関数を呼び出し
  pcl::fromROSMsg(pointcloud_, *pointcloud_pcl);
  // pointcloud_を元にcostmapを生成
  // グリッドマップの初期化

  std::vector<std::vector<float>> costmap(GRID_WIDTH_, std::vector<float>(GRID_HEIGHT_, 0.0));

  // 点群をグリッドマップに変換
  for (const auto & point : pointcloud_pcl->points) {
    int x_index = static_cast<int>(point.x / GRID_RESOLUTION_);
    int y_index = static_cast<int>((point.y + GRID_WIDTH_ / 2) / GRID_RESOLUTION_);
    const int KERNEL_SIZE = 1;            // 評価関数を設定する範囲（カーネルサイズ）
    const float SURROUNDING_COST = 50.0;  // 周囲の格子の評価関数
    if (x_index >= 0 && x_index < GRID_WIDTH_ && y_index >= 0 && y_index < GRID_HEIGHT_) {
      costmap[x_index][y_index] += 100.0;  // 点が存在する格子は評価関数を高く設定
      for (int x = x_index - KERNEL_SIZE; x <= x_index + KERNEL_SIZE; ++x) {
        for (int y = y_index - KERNEL_SIZE; y <= y_index + KERNEL_SIZE; ++y) {
          if (x >= 0 && x < GRID_WIDTH_ && y >= 0 && y < GRID_HEIGHT_) {
            costmap[x][y] += SURROUNDING_COST;
          }
        }
      }
    }
  }

  const float REFERENCE_TRAJECTORY_COST = -1;  // reference_trajectoryの格子の評価関数
  for (const auto & point : reference_trajectory_.points) {
    int x_index = static_cast<int>(point.pose.position.x / GRID_RESOLUTION_);
    int y_index = static_cast<int>((point.pose.position.y + GRID_WIDTH_ / 2) / GRID_RESOLUTION_);

    if (x_index >= 0 && x_index < GRID_WIDTH_ && y_index >= 0 && y_index < GRID_HEIGHT_) {
      costmap[x_index][y_index] += REFERENCE_TRAJECTORY_COST;
    }
  }

  return costmap;
}

SampleNode::Trajectory SampleNode::evaluate_trajectory(
  const std::vector<SampleNode::Trajectory> & trajectory_set, const std::vector<std::vector<float>> & costmap)
{
  // RCLCPP_INFO((this->get_logger(), "Evaluating trajectories.");
  Trajectory best_trajectory;
  std::vector<float> trajectory_cost(trajectory_set.size(), 0.0f);
  size_t index = 0;

  // trajectory_setをcostmapで評価し、最適な軌道を選択
  for (const auto & trajectory_candidate : trajectory_set) {
    // RCLCPP_INFO(this->get_logger(), "Evaluating trajectory %zu", index);

    // trajectoryをcostmapで評価
    for (const auto & trajectory_point : trajectory_candidate.points) {
      int x_index = static_cast<int>(trajectory_point.pose.position.x / GRID_RESOLUTION_);
      int y_index = static_cast<int>((trajectory_point.pose.position.y + GRID_WIDTH_ / 2) / GRID_RESOLUTION_);
      // RCLCPP_INFO(this->get_logger(), "x_index=%d, y_index=%d", x_index, y_index);

      if (x_index >= 0 && x_index < GRID_WIDTH_ && y_index >= 0 && y_index < GRID_HEIGHT_) {
        trajectory_cost[index] += costmap[x_index][y_index];
      } else {
        RCLCPP_WARN(
          this->get_logger(), "Trajectory point out of costmap bounds: x_index=%d, y_index=%d", x_index, y_index);
      }
      /*
            RCLCPP_INFO(
              this->get_logger(), "Trajectory point (%f, %f) evaluated with cost %f", trajectory_point.pose.position.x,
              trajectory_point.pose.position.y, costmap[x_index][y_index]);
              */
    }

    // RCLCPP_INFO(this->get_logger(), "Total cost for trajectory %zu: %f", index, trajectory_cost[index]);
    index++;
  }

  // 最小コストのインデックスを出力
  int min_index =
    std::distance(trajectory_cost.begin(), std::min_element(trajectory_cost.begin(), trajectory_cost.end()));
  best_trajectory = trajectory_set[min_index];

  // RCLCPP_INFO(this->get_logger(), "Best trajectory selected: %d with cost %f", min_index,
  // trajectory_cost[min_index]);

  return best_trajectory;
}

double SampleNode::quaternionToInclination(Eigen::Quaterniond q)
{
  double inclination = 2.0 * q.z() / q.w();
  return inclination;
}

Eigen::Vector3d SampleNode::quaternionToVector(Eigen::Quaterniond q)
{
  Eigen::Vector3d unitVector(1, 0, 0);
  Eigen::Vector3d directionVector = q * unitVector;
  // 計算された方向ベクトルをログに記録
  /*RCLCPP_WARN(
    this->get_logger(), "Direction Vector: [%f, %f, %f]", directionVector.x(), directionVector.y(),
    directionVector.z());
  */
  return directionVector;
}
// エルミート補間関数
std::vector<geometry_msgs::msg::Point> SampleNode::hermiteInterpolate(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1, Eigen::Vector3d m0, Eigen::Vector3d m1,
  int numPoints)
{
  // RCLCPP_INFO(this->get_logger(), "Performing Hermite interpolation.");
  std::vector<geometry_msgs::msg::Point> interpolatedPoints;

  Eigen::Vector3d v0 = pointToVector3d(p0);
  Eigen::Vector3d v1 = pointToVector3d(p1);

  // Control points
  Eigen::Vector3d c0 = v0;
  Eigen::Vector3d c1 = v0 + m0 * 3;
  Eigen::Vector3d c2 = v1 - m1 * 3;
  Eigen::Vector3d c3 = v1;

  for (int i = 0; i <= numPoints; ++i) {
    double t = static_cast<double>(i) / numPoints;
    double t2 = t * t;
    double t3 = t2 * t;
    double one_minus_t = 1.0 - t;
    double one_minus_t2 = one_minus_t * one_minus_t;
    double one_minus_t3 = one_minus_t2 * one_minus_t;

    Eigen::Vector3d point = one_minus_t3 * c0 + 3.0 * one_minus_t2 * t * c1 + 3.0 * one_minus_t * t2 * c2 + t3 * c3;

    interpolatedPoints.push_back(vector3dToPoint(point));
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
