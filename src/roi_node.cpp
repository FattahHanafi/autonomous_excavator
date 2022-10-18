#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"

using std::placeholders::_1;

class RegionOfInterestNode : public rclcpp::Node {
  public:
    explicit RegionOfInterestNode() : Node("roi_node")
    {
        roi_subscriber =
            this->create_subscription<sensor_msgs::msg::RegionOfInterest>("camera/roi", 10, std::bind(&RegionOfInterestNode::roi_callback, this, _1));

        raw_depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/raw_depth_image", 10, std::bind(&RegionOfInterestNode::raw_depth_image_callback, this, _1));
        cropped_raw_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/cropped_raw_depth_image", 10);

        filtered_depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/filtered_depth_image", 10, std::bind(&RegionOfInterestNode::filtered_depth_image_callback, this, _1));
        cropped_filtered_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/cropped_filtered_depth_image", 10);

        header.frame_id = "camera_depth_optical_frame";
    }

  private:
    sensor_msgs::msg::RegionOfInterest roi;
    std_msgs::msg::Header header = std_msgs::msg::Header();
    sensor_msgs::msg::Image::SharedPtr cropped_raw_depth_image_message;
    sensor_msgs::msg::Image::SharedPtr cropped_filtered_depth_image_message;

    void raw_depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!roi.width || !roi.height) {
            // cropped_raw_depth_imgae_publisher->publish(*msg);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat roi_depth_data =
            cv_ptr->image(cv::Range(roi.y_offset, roi.y_offset + roi.height), cv::Range(roi.x_offset, roi.x_offset + roi.width));

        header.stamp = this->get_clock().get()->now();

        cropped_raw_depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, roi_depth_data).toImageMsg();
        cropped_raw_depth_imgae_publisher->publish(*cropped_raw_depth_image_message);
    }

    void filtered_depth_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!roi.width || !roi.height) {
            // cropped_filtered_depth_imgae_publisher->publish(*msg);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat roi_depth_data =
            cv_ptr->image(cv::Range(roi.y_offset, roi.y_offset + roi.height), cv::Range(roi.x_offset, roi.x_offset + roi.width));

        header.stamp = this->get_clock().get()->now();

        cropped_filtered_depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, roi_depth_data).toImageMsg();
        cropped_filtered_depth_imgae_publisher->publish(*cropped_filtered_depth_image_message);
    }

    void roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        roi.x_offset = msg->x_offset;
        roi.y_offset = msg->y_offset;
        roi.width = msg->width;
        roi.height = msg->height;
        roi.do_rectify = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_subscriber;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_depth_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_raw_depth_imgae_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr filtered_depth_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_filtered_depth_imgae_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RegionOfInterestNode>());
    rclcpp::shutdown();

    return 0;
}
