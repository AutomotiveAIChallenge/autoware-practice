#ifndef DUMMY_LOCALIZER_HPP
#define DUMMY_LOCALIZER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class DummyLocalizer : public rclcpp::Node
{
public:
    DummyLocalizer();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void publish_odometry();

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    std::optional<geometry_msgs::msg::Pose> last_pose_;
    std::optional<geometry_msgs::msg::Twist> last_twist_;
};

#endif // DUMMY_LOCALIZER_HPP
