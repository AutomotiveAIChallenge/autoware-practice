#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class DummyLocalizer : public rclcpp::Node
{
public:
    DummyLocalizer() : Node("dummy_localizer")
    {
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
        pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/simulator/ground_truth/pose", 10, std::bind(&DummyLocalizer::pose_callback, this, std::placeholders::_1));
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/simulator/ground_truth/twist", 10, std::bind(&DummyLocalizer::twist_callback, this, std::placeholders::_1));
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_pose_ = msg->pose;
        publish_odometry();
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        last_twist_ = msg->twist;
        publish_odometry();
    }

    void publish_odometry()
    {
        if (last_pose_ && last_twist_) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = this->get_clock()->now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose = *last_pose_;
            odom.twist.twist = *last_twist_;
            publisher_->publish(odom);
            last_pose_.reset();
            last_twist_.reset();
        }
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    std::optional<geometry_msgs::msg::Pose> last_pose_;
    std::optional<geometry_msgs::msg::Twist> last_twist_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DummyLocalizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
