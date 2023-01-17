#include <math.h>
#include <sys/types.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

class SurfaceReconstructor : public rclcpp::Node {
  public:
    explicit SurfaceReconstructor() : Node("surface_reconstrcutor")
    {
        m_marker_message.ns = "soil_surface";
        m_marker_message.id = 0;
        m_marker_message.header.frame_id = "camera";
        m_marker_message.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        m_marker_message.action = visualization_msgs::msg::Marker::ADD;
        m_marker_message.pose.position.x = 0;
        m_marker_message.pose.position.y = 0;
        m_marker_message.pose.position.z = 0;
        m_marker_message.pose.orientation.x = 0;
        m_marker_message.pose.orientation.y = 0;
        m_marker_message.pose.orientation.z = 0;
        m_marker_message.pose.orientation.w = 1;
        m_marker_message.color.r = 1;
        m_marker_message.color.a = 1;
        m_marker_message.scale.x = 1;
        m_marker_message.scale.y = 1;
        m_marker_message.scale.z = 1;

        pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("Camera/PointCloud", 10,
                                                                                 std::bind(&SurfaceReconstructor::pc_callback, this, _1));
        surface_publisher = this->create_publisher<visualization_msgs::msg::Marker>("Surface/Reconstructed", 10);
    }

  private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::PointCloud2ConstIterator<float> it_top(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_bottom(*msg, "x");

		const uint32_t width = msg->width;
		const uint32_t height = msg->height;

        m_marker_message.points.clear();
		it_top += width;

        geometry_msgs::msg::Point top_point;
        geometry_msgs::msg::Point bottom_point;
        for (uint32_t i = 0; i < (height - 1); ++i) {
            if (i != 0) {
                ++it_top;
                ++it_bottom;
            }
            for (uint32_t j = 0; j < (width - 1); ++j) {
                bottom_point.x = it_bottom[0];
                bottom_point.y = it_bottom[1];
                bottom_point.z = it_bottom[2];
                m_marker_message.points.push_back(bottom_point);
                top_point.x = it_top[0];
                top_point.y = it_top[1];
                top_point.z = it_top[2];
                m_marker_message.points.push_back(top_point);
                ++it_bottom;
                bottom_point.x = it_bottom[0];
                bottom_point.y = it_bottom[1];
                bottom_point.z = it_bottom[2];
                m_marker_message.points.push_back(bottom_point);

				m_marker_message.points.push_back(top_point);
                ++it_top;
                top_point.x = it_top[0];
                top_point.y = it_top[1];
                top_point.z = it_top[2];
                m_marker_message.points.push_back(top_point);
                m_marker_message.points.push_back(bottom_point);
            }
        }

        m_marker_message.header.stamp = this->get_clock()->now();
        surface_publisher->publish(m_marker_message);
    }

    visualization_msgs::msg::Marker m_marker_message;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr surface_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurfaceReconstructor>());
    rclcpp::shutdown();

    return 0;
}
