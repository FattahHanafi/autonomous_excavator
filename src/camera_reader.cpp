#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <librealsense2/rs.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <variant>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthImagePublisher : public rclcpp::Node {
  public:
    explicit DepthImagePublisher() : Node("camera_reader")
    {
        raw_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/raw_depth_image", 10);
        filtered_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/filtered_depth_image", 10);

        this->declare_parameter("decimation_filter", false);
        this->declare_parameter("decimation_magnitude", 2);

		this->declare_parameter("spatial_filter", false);
        this->declare_parameter("spatial_magnitude", 2);
        this->declare_parameter("spatial_smooth_alpha", 0.5f);
        this->declare_parameter("spatial_smooth_delta", 20);
        // this->declare_parameter("spatial_hole_filling", 0);
        
		this->declare_parameter("temporial_filter", false);
        this->declare_parameter("temporial_smooth_alpha", 0.4f);
        this->declare_parameter("temporial_smooth_delta", 20);
        // this->declare_parameter("temporial_persistency_index", 3);
        
		this->declare_parameter("hole_filling_filter", false);
        this->declare_parameter("hole_filling", 1);

        this->declare_parameter("threshold_filter", false);
        this->declare_parameter("threshold_min_distance", 0.0f);
        this->declare_parameter("threshold_max_distance", 4.0f);

        filters.reserve(4);

        header.frame_id = "camera_depth_optical_frame";

        pipe.start();

        timer_ = this->create_wall_timer(100ms, std::bind(&DepthImagePublisher::captureImage, this));
    }

  private:
    sensor_msgs::msg::Image::SharedPtr depth_image_message;
    sensor_msgs::msg::Image::SharedPtr filtered_depth_image_message;
    std_msgs::msg::Header header;
    rs2::pipeline pipe;
    rs2::frameset frameset;

    std::vector<rs2::filter> filters;
    rs2::decimation_filter dec_filter;
    rs2::spatial_filter spa_filter;
    rs2::temporal_filter tmp_filter;
    rs2::hole_filling_filter hlf_filter;
    rs2::threshold_filter trh_filter;

    void captureImage()
    {
        frameset = pipe.wait_for_frames();

        auto depth_frame = frameset.get_depth_frame();
        auto color_frame = frameset.get_color_frame();

        cv::Mat depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        header.stamp = this->get_clock().get()->now();
        depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_data).toImageMsg();
        raw_depth_image_publisher->publish(*depth_image_message);

        // Set filter values
        int dec_mag = this->get_parameter("decimation_magnitude").get_parameter_value().get<int>();
        dec_mag = (dec_mag > 8) ? 8 : dec_mag;
        dec_mag = (dec_mag < 2) ? 2 : dec_mag;
        dec_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_mag);

        int spa_mag = this->get_parameter("spatial_magnitude").get_parameter_value().get<int>();
        spa_mag = (spa_mag > 5) ? 5 : spa_mag;
        spa_mag = (spa_mag < 1) ? 1 : spa_mag;
        float spa_sma = this->get_parameter("spatial_smooth_alpha").get_parameter_value().get<float>();
        spa_sma = (spa_sma > 1.00) ? 1.00 : spa_sma;
        spa_sma = (spa_sma < 0.25) ? 0.25 : spa_sma;
        int spa_smd = this->get_parameter("spatial_smooth_delta").get_parameter_value().get<int>();
        spa_smd = (spa_smd > 50) ? 50 : spa_smd;
        spa_smd = (spa_smd < 1) ? 1 : spa_smd;
        // int spa_hlf = this->get_parameter("spatial_hole_filling").get_parameter_value().get<int>();
        // spa_hlf = (spa_hlf > 5) ? 5 : spa_hlf;
        // spa_hlf = (spa_hlf < 0) ? 0 : spa_hlf;
        spa_filter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spa_mag);
        spa_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spa_sma);
        spa_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spa_smd);

        float tmp_sma = this->get_parameter("temporial_smooth_alpha").get_parameter_value().get<float>();
        tmp_sma = (tmp_sma > 1.00) ? 1.00 : tmp_sma;
        tmp_sma = (tmp_sma < 0.00) ? 0.00 : tmp_sma;
        int tmp_smd = this->get_parameter("temporial_smooth_delta").get_parameter_value().get<int>();
        tmp_smd = (tmp_smd > 20) ? 20 : tmp_smd;
        tmp_smd = (tmp_smd < 1) ? 1 : tmp_smd;
        // int tmp_psi = this->get_parameter("temporial_persistency_index").get_parameter_value().get<int>();
        // tmp_psi = (tmp_psi > 20) ? 20 : tmp_psi;
        // tmp_psi = (tmp_psi < 1) ? 1 : tmp_psi;
        tmp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, tmp_sma);
        tmp_filter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, tmp_smd);

        /* int hlf_hlf = this->get_parameter("hole_filling").get_parameter_value().get<int>();
                hlf_filer.supports
        hlf_hlf = (hlf_hlf > 2) ? 2 : hlf_hlf;
        hlf_hlf = (hlf_hlf < 0) ? 0 : hlf_hlf;
        hlf_filter.set_option(RS2_OPTION_FILTER_OPTION, hlf_hlf); */

        float trh_min = this->get_parameter("threshold_min_distance").get_parameter_value().get<float>();
        float trh_max = this->get_parameter("threshold_max_distance").get_parameter_value().get<float>();
        trh_min = (trh_min < 0) ? 0 : trh_min;
        trh_max = (trh_max < trh_min) ? trh_min : trh_max;
        trh_filter.set_option(RS2_OPTION_MIN_DISTANCE, trh_min);
        trh_filter.set_option(RS2_OPTION_MAX_DISTANCE, trh_max);

        // Declare filters
        filters.clear();
        if (this->get_parameter("decimation_filter").get_parameter_value().get<bool>()) filters.push_back(dec_filter);
        if (this->get_parameter("spatial_filter").get_parameter_value().get<bool>()) filters.push_back(spa_filter);
        if (this->get_parameter("temporial_filter").get_parameter_value().get<bool>()) filters.push_back(tmp_filter);
        // if (this->get_parameter("hole_filling_filter").get_parameter_value().get<bool>()) filters.push_back(hlf_filter);
        if (this->get_parameter("threshold_filter").get_parameter_value().get<bool>()) filters.push_back(trh_filter);

        for (auto filter : filters) depth_frame = depth_frame.apply_filter(filter);

        cv::Mat filtered_depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void*)depth_frame.get_data(),
                                    cv::Mat::AUTO_STEP);

        filtered_depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, filtered_depth_data).toImageMsg();

        filtered_depth_image_publisher->publish(*filtered_depth_image_message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_image_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImagePublisher>());
    rclcpp::shutdown();

    return 0;
}
