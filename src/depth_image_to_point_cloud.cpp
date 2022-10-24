#include <cstdint>
#include <cstring>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/point_field__struct.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using std::placeholders::_1;

class DepthImageToPointCloud : public rclcpp::Node {
  public:
    explicit DepthImageToPointCloud() : Node("depth_image_to_point_cloud")
    {
        pipe.start();

        frame_set = pipe.wait_for_frames();

        pipe.stop();

        // header.frame_id = "camera_depth_optical_frame";
        header.frame_id = "map";

        roi_subscriber = this->create_subscription<sensor_msgs::msg::RegionOfInterest>("camera/roi", 10,
                                                                                       std::bind(&DepthImageToPointCloud::roi_callback, this, _1));

        depth_image_subsciber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/reconstructed_cropped_filtered_depth_image", 10, std::bind(&DepthImageToPointCloud::depth_image_callback, this, _1));

        point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/point_cloud", 10);
    }

  private:
    rs2::pipeline pipe;
    rs2::frameset frame_set;
    rs2::pointcloud point_cloud;

    sensor_msgs::msg::RegionOfInterest roi;
    sensor_msgs::msg::PointCloud2 point_cloud_message;
    std_msgs::msg::Header header;

    void depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        header.stamp = this->get_clock().get()->now();
        uint16_t* data_ptr = (uint16_t*)frame_set.get_depth_frame().get_data();

        uint32_t idx2 = 0;
        for (uint32_t i = roi.y_offset; i < roi.y_offset + roi.height; ++i) {
            for (uint32_t j = roi.x_offset; j < roi.x_offset + roi.width; j++) {
                uint32_t idx = i * frame_set.get_depth_frame().get_width() + j;
                memcpy(&data_ptr[idx2], &msg->data[2 * idx], 2);
                idx2++;
            }
        }

        point_cloud.calculate(frame_set.get_depth_frame());

        rs2::points dd = point_cloud.calculate(frame_set.get_depth_frame());

        point_cloud_message.header = header;
        point_cloud_message.height = msg->height;
        point_cloud_message.width = msg->width;
        point_cloud_message.point_step = sizeof(rs2::vertex);
        point_cloud_message.row_step = msg->width * point_cloud_message.point_step;
        point_cloud_message.is_dense = true;
        point_cloud_message.data.resize(msg->height * point_cloud_message.row_step);

		sensor_msgs::msg::PointField point_field;
		point_field.count = 1;
		point_field.datatype = 1;
		point_field.offset = 0;
		point_field.name = "Alaki";

		point_cloud_message.fields.push_back(point_field);
        RCLCPP_INFO(this->get_logger(), "here we go again! %li", sizeof(point_cloud_message.data[0]));


        idx2 = 0;
        for (uint32_t i = roi.y_offset; i < roi.y_offset + roi.height; ++i) {
            for (uint32_t j = roi.x_offset; j < roi.x_offset + roi.width; j++) {
                uint32_t idx = i * frame_set.get_depth_frame().get_width() + j;
                RCLCPP_INFO(this->get_logger(), "index %u, %u", i, j);
                memcpy(&point_cloud_message.data[idx2], dd.get_vertices() + (idx * sizeof(rs2::vertex)), sizeof(rs2::vertex));
                idx2 += sizeof(rs2::vertex);
            }
        }

        point_cloud_publisher->publish(point_cloud_message);
    }

    void roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        roi.x_offset = msg->x_offset;
        roi.y_offset = msg->y_offset;
        roi.width = msg->width;
        roi.height = msg->height;
        roi.do_rectify = true;
		RCLCPP_INFO(this->get_logger(), "ROI set = %u, %u, %u, %u", roi.x_offset, roi.width, roi.y_offset, roi.height);
    }

    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_subscriber;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_subsciber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImageToPointCloud>());
    rclcpp::shutdown();

    return 0;
}

