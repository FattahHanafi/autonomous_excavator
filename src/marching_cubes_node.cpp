#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <geometry_msgs/msg/point32.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "../include/Blade.h"
#include "../include/MarchingCubes.h"
#include "../include/Vec3.h"

MarchingCubes mc = MarchingCubes(135.0f, 60.0f, 60.0f, 0.01f, 0.01f, 0.01f);

using std::placeholders::_1;

class MarchingCubesPublisher : public rclcpp::Node {
  public:
    explicit MarchingCubesPublisher() : Node("marching_cubes_node")
    {
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        m_marker_message.ns = "mc_cubes";
        m_marker_message.id = 0;
        m_marker_message.header.frame_id = "container";
        m_marker_message.type = 11;
        m_marker_message.action = 0;
        m_marker_message.pose.position.x = 0;
        m_marker_message.pose.position.y = 0;
        m_marker_message.pose.position.z = 0;
        m_marker_message.pose.orientation.x = 0;
        m_marker_message.pose.orientation.y = 0;
        m_marker_message.pose.orientation.z = 0;
        m_marker_message.pose.orientation.w = 1;
        m_marker_message.color.r = 1;
		m_marker_message.color.g = 0.8;
		m_marker_message.color.b = 0;
        m_marker_message.color.a = 1;
        m_marker_message.scale.x = 1;
        m_marker_message.scale.y = 1;
        m_marker_message.scale.z = 1;

        m_pc_rotated_message.header.frame_id = "container";
        pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("Camera/PointCloud", 10,
                                                                                 std::bind(&MarchingCubesPublisher::pc_callback, this, _1));
        blade_subscriber =
            this->create_subscription<geometry_msgs::msg::Polygon>("Machine/Blade", 10, std::bind(&MarchingCubesPublisher::blade_callback, this, _1));

        update_subsciber =
            this->create_subscription<std_msgs::msg::Empty>("MarchingCubes/Update", 10, std::bind(&MarchingCubesPublisher::UpdateSurface, this, _1));

        marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("MarchingCubes/Cubes", 10);
    }

  private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!m_UpdateSurface) return;
        m_UpdateSurface = false;
        auto t = tf_buffer->lookupTransform(m_pc_rotated_message.header.frame_id, msg->header.frame_id, tf2::TimePointZero);

        m_pc_rotated_message.header.stamp = msg->header.stamp;
        m_pc_rotated_message.set__fields(msg->fields);
        m_pc_rotated_message.set__height(msg->height);
        m_pc_rotated_message.set__width(msg->width);
        m_pc_rotated_message.set__is_bigendian(msg->is_bigendian);
        m_pc_rotated_message.set__is_dense(msg->is_dense);
        m_pc_rotated_message.set__point_step(msg->point_step);
        m_pc_rotated_message.set__row_step(msg->row_step);
        m_pc_rotated_message.set__data(msg->data);

        geometry_msgs::msg::Point p_in;
        geometry_msgs::msg::Point p_out;

        sensor_msgs::PointCloud2Iterator<float> xyz_it(*msg, "x");
        auto start = std::chrono::steady_clock::now();
        while (xyz_it != xyz_it.end()) {
            p_in.x = xyz_it[0];
            p_in.y = xyz_it[1];
            p_in.z = xyz_it[2];
            tf2::doTransform(p_in, p_out, t);
            xyz_it[0] = float(p_out.x);
            xyz_it[1] = float(p_out.y);
            xyz_it[2] = float(p_out.z);
            ++xyz_it;
        }
        auto end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Rotation took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        geometry_msgs::msg::Point p1;
        geometry_msgs::msg::Point p2;

        start = std::chrono::steady_clock::now();
        for (uint32_t i = 0; i < mc.m_Step.x; ++i) {
            p1.x = i * mc.m_Size.x;
            RCLCPP_INFO(this->get_logger(), "%u - %f", i, mc.m_Step.x);
            for (uint32_t j = 0; j < mc.m_Step.y; ++j) {
                p1.y = j * mc.m_Size.y;
                float dis = 1e6;
                float dis2;
                sensor_msgs::PointCloud2Iterator<float> xyz_it2(*msg, "x");
                while (xyz_it2 != xyz_it2.end()) {
                    dis2 = (p1.x - xyz_it2[0]) * (p1.x - xyz_it2[0]);
                    dis2 += (p1.y - xyz_it2[1]) * (p1.y - xyz_it2[1]);
                    if (dis2 < dis) {
                        dis = dis2;
                        p1.z = xyz_it2[2];
                    }
                    ++xyz_it2;
                }
                for (uint32_t k = 0; k < mc.m_Step.z; ++k) {
                    float z = k * mc.m_Size.z;
                    mc.SetVertex(i, j, k, z < p1.z);
                }
            }
        }

        end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Triangulation took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        start = std::chrono::steady_clock::now();
        mc.RebuildCubes();
        end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Rebuilding took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        Publish();
    }

    void blade_callback(const geometry_msgs::msg::Polygon::SharedPtr msg) { mc.SetBlade(msg); }

    void UpdateSurface(const std_msgs::msg::Empty msg)
    {
        RCLCPP_INFO(this->get_logger(), "Update requested!");
        m_UpdateSurface = true;
    }

    void Publish()
    {
        m_marker_message.header.stamp = this->get_clock()->now();
        mc.CreateMessage(&m_marker_message);
        marker_publisher->publish(m_marker_message);
    }
    bool m_UpdateSurface = false;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr blade_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_subsciber;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;

    geometry_msgs::msg::Point32::SharedPtr m_point32;
    geometry_msgs::msg::Polygon::SharedPtr m_polygon_message;
    sensor_msgs::msg::PointCloud2 m_pc_rotated_message;
    visualization_msgs::msg::Marker m_marker_message;

    sensor_msgs::msg::PointCloud2 pc_rotated;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
};

int main(int argc, char *argv[])
{
    mc.RebuildCubes();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchingCubesPublisher>());
    rclcpp::shutdown();

    return 0;
}
