#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/empty.hpp>
#include <vector>

#include "../include/Blade.h"
#include "../include/MarchingCubes.h"
#include "../include/Vec3.h"

MarchingCubes mc;

using std::placeholders::_1;

class MarchingCubesPublisher : public rclcpp::Node {
  public:
    explicit MarchingCubesPublisher() : Node("marching_cubes_node")
    {
        pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("Camera/PointCloud", 10,
                                                                                 std::bind(&MarchingCubesPublisher::pc_callback, this, _1));
        blade_subscriber =
            this->create_subscription<geometry_msgs::msg::Polygon>("Machine/Blade", 10, std::bind(&MarchingCubesPublisher::blade_callback, this, _1));

        update_subsciber =
            this->create_subscription<std_msgs::msg::Empty>("UpdateSurface", 10, std::bind(&MarchingCubesPublisher::UpdateSurface, this, _1));
    }

  private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
		RCLCPP_INFO(this->get_logger(), "HOORAAAA!");
        if (m_UpdateSurface) {
            m_UpdateSurface = false;
            mc.RebuildSurface(msg);
        }
    }

    void blade_callback(const geometry_msgs::msg::Polygon::SharedPtr msg) { mc.SetBlade(msg); }

    void UpdateSurface(const std_msgs::msg::Empty::SharedPtr msg) { m_UpdateSurface = true; }

    bool m_UpdateSurface = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr blade_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_subsciber;

    geometry_msgs::msg::Point32::SharedPtr m_point32;
    geometry_msgs::msg::Polygon::SharedPtr m_polygon_message;
};

int main(int argc, char* argv[])
{
    mc = MarchingCubes(200, 100, 100, 0.01f, 0.01f, 0.01f, 2000.0f, -500.0f, -1000.0f);
    mc.RebuildCubes();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchingCubesPublisher>());
    rclcpp::shutdown();

    return 0;
}
