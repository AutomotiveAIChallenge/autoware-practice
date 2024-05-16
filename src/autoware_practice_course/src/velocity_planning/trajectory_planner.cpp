#include "trajectory_planner.hpp"

#include <memory>
#include <cmath>

namespace autoware_practice_course
{

SampleNode::SampleNode() : Node("trajectory_planner")
{
  using std::placeholders::_1;

  goal_ = 100.0;
  declare_parameter<double>("goal", 100.0);
  get_parameter("goal", goal_);
  pub_trajectory_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", rclcpp::QoS(1));
  sub_kinematic_state_ = create_subscription<Odometry>("/localization/kinematic_state", rclcpp::QoS(1), std::bind(&SampleNode::update_vehicle_position, this, _1));

  const auto period = rclcpp::Rate(10).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this] { on_timer(); });
}

void SampleNode::update_vehicle_position(const Odometry & msg)
{
  position_x_ = msg.pose.pose.position.x;
}

void SampleNode::on_timer()
{
  const auto stamp = now();

  Trajectory trajectory;
  trajectory.header.stamp = stamp;
  trajectory.header.frame_id = "map";
  int distance = static_cast<int>(std::floor(goal_ - position_x_));

  for (int i = 1; i <= distance; ++i) {
    TrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(i);
    point.time_from_start.nanosec = 0;
    point.pose.position.x = static_cast<double>(i);
    point.pose.position.y = 0.0;
    point.pose.position.z = 0.0; 
    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;
    double waypoint_i_x = static_cast<double>(i) + position_x_;
    point.longitudinal_velocity_mps = (waypoint_i_x < 50) ? (0.2 * waypoint_i_x) : (-0.2 * waypoint_i_x + 20.0);
    point.lateral_velocity_mps = 0.0;
    point.acceleration_mps2 = 0.0;
    point.heading_rate_rps = 0.0;
    point.front_wheel_angle_rad = 0.0;
    point.rear_wheel_angle_rad = 0.0;
    trajectory.points.push_back(point);

  }
  
  pub_trajectory_->publish(trajectory);
}

} 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autoware_practice_course::SampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
