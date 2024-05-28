#ifndef TRAJECTORY_LOADER_HPP_
#define TRAJECTORY_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>


namespace autoware_practice_course
{

class SampleNode : public rclcpp::Node
{
public:
  SampleNode();

private:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

  void on_timer();
  void load_path(const std::string & file_path);
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  Trajectory trajectory_;

};

} 

#endif
