#ifndef DUMMY_LOCALIZER_HPP
#define DUMMY_LOCALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
namespace autoware_practice_course
{
class SampleNode : public rclcpp::Node
{
public:
    SampleNode();

private:
    using Odometry = nav_msgs::msg::Odometry;
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    using TwistStamped = geometry_msgs::msg::TwistStamped;
    using Pose = geometry_msgs::msg::Pose;
    using Twist = geometry_msgs::msg::Twist;

    void pose_callback(const PoseStamped::SharedPtr msg);
    void twist_callback(const TwistStamped::SharedPtr msg);
    void publish_odometry();

    rclcpp::Publisher<Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<TwistStamped>::SharedPtr twist_subscriber_;
    Pose last_pose_;
    Twist last_twist_;
    bool has_pose_;
    bool has_twist_;
};
}
#endif // DUMMY_LOCALIZER_HPP
