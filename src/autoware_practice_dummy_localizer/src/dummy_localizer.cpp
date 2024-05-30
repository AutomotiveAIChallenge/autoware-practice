#include "dummy_localizer.hpp"

DummyLocalizer::DummyLocalizer() : Node("dummy_localizer")
{
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/simulator/ground_truth/pose", 10, std::bind(&DummyLocalizer::pose_callback, this, std::placeholders::_1));
    twist_subscriber_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/simulator/ground_truth/twist", 10, std::bind(&DummyLocalizer::twist_callback, this, std::placeholders::_1));
}

void DummyLocalizer::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_pose_ = msg->pose;
    publish_odometry();
}

void DummyLocalizer::twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    last_twist_ = msg->twist;
    publish_odometry();
}

void DummyLocalizer::publish_odometry()
{
    if (last_pose_ && last_twist_) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose = *last_pose_;
        odom.twist.twist = *last_twist_;
        publisher_->publish(odom);
        last_pose_.reset();
        last_twist_.reset();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
