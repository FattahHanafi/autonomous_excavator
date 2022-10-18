#include <GLFW/glfw3.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <librealsense2/rs.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class DepthImagePublisher : public rclcpp::Node {
  public:
    explicit DepthImagePublisher() : Node("camera_reader")
    {
        raw_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/raw_depth_image", 10);
        filtered_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/filtered_depth_image", 10);

        header.frame_id = "camera_depth_optical_frame";

        pipe.start();

        while (rclcpp::ok()) {
            captureImage();
        }
    }

  private:
    sensor_msgs::msg::Image::SharedPtr depth_image_message;
    std_msgs::msg::Header header;
    rs2::pipeline pipe;
    rs2::frameset frameset;

    void captureImage()
    {
        frameset = pipe.wait_for_frames();

        auto depth_frame = frameset.get_depth_frame();
        auto color_frame = frameset.get_color_frame();

        cv::Mat depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        header.stamp = this->get_clock().get()->now();
        depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_data).toImageMsg();
        raw_depth_imgae_publisher->publish(*depth_image_message);
        filtered_depth_imgae_publisher->publish(*depth_image_message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_depth_imgae_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_imgae_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImagePublisher>());
    rclcpp::shutdown();

    return 0;
}
