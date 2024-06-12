#ifndef SIMPLE_LIDAR_SIMULATOR_HPP
#define SIMPLE_LIDAR_SIMULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace autoware_practice_lidar_simulator
{
class SampleNode : public rclcpp::Node
{
public:
    SampleNode();

private:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
    
    void on_timer();
    PointCloudXYZ::Ptr create_object_point_cloud(float x_center, float y_center, float width, float height, float resolution);
    PointCloudXYZ::Ptr filter_points_within_radius(PointCloudXYZ::Ptr cloud, float radius);
    std::vector<std::pair<float, float>> read_object_centers_from_csv(const std::string& file_path);

    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<float, float>> object_centers_;

};

}

#endif  // SIMPLE_LIDAR_SIMULATOR_HPP
