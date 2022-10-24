#include <cv_bridge/cv_bridge.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_vector_long_double.h>
#include <stdio.h>

#include <chrono>
#include <librealsense2/rs.hpp>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "std_msgs/msg/header.hpp"

#define NCOEFFS 12
#define NBREAKS NCOEFFS - 2

gsl_bspline_workspace *bw;
gsl_vector *B;
gsl_vector *c, *w;
gsl_vector *x, *y;
gsl_matrix *X, *cov;
gsl_multifit_linear_workspace *mw;

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthImagePublisher : public rclcpp::Node {
  public:
    explicit DepthImagePublisher() : Node("camera_reader")
    {
        roi_subscriber =
            this->create_subscription<sensor_msgs::msg::RegionOfInterest>("camera/roi", 10, std::bind(&DepthImagePublisher::roi_callback, this, _1));

        raw_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/raw_depth_image", 10);
        filtered_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/filtered_depth_image", 10);
        cropped_filtered_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("camera/cropped_filtered_depth_image", 10);

        reconstructed_cropped_filtered_depth_image_publisher =
            this->create_publisher<sensor_msgs::msg::Image>("camera/reconstructed_cropped_filtered_depth_image", 10);

        point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/point_cloud", 10);

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

        timer_ = this->create_wall_timer(1000ms, std::bind(&DepthImagePublisher::captureImage, this));
    }

  private:
    sensor_msgs::msg::RegionOfInterest roi;
    sensor_msgs::msg::Image::SharedPtr depth_image_message;
    sensor_msgs::msg::Image::SharedPtr filtered_depth_image_message;
    sensor_msgs::msg::Image::SharedPtr cropped_filtered_depth_image_message;
    sensor_msgs::msg::Image::SharedPtr reconstructed_cropped_filtered_depth_image_message;

    sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_message;

    std_msgs::msg::Header header;
    rs2::pipeline pipe;
    rs2::frameset frameset;
    rs2::pointcloud point_cloud;

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

        cv::Mat depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

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

        cv::Mat filtered_depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(),
                                    cv::Mat::AUTO_STEP);

        filtered_depth_image_message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, filtered_depth_data).toImageMsg();

        filtered_depth_image_publisher->publish(*filtered_depth_image_message);

        if (!roi.width || !roi.height) return;

        auto data_ptr = (uint16_t *)depth_frame.get_data();

        int idx = 0;
        for (int i = 0; i < depth_frame.get_height(); ++i)
            for (int j = 0; j < depth_frame.get_width(); ++j) {
                if ((i < (int)roi.y_offset) || (i > (int)(roi.y_offset + roi.height)) || (j < (int)roi.x_offset) ||
                    (j > (int)(roi.x_offset + roi.width)))
                    data_ptr[idx] = 0;
                idx++;
            }

        cv::Mat cropped_filtered_depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U, (void *)depth_frame.get_data(),
                                            cv::Mat::AUTO_STEP);

        cropped_filtered_depth_image_message =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, cropped_filtered_depth_data).toImageMsg();

        cropped_filtered_depth_imgae_publisher->publish(*cropped_filtered_depth_image_message);

        // Allocate enough memory space
        x = gsl_vector_alloc(roi.width);
        y = gsl_vector_alloc(roi.width);
        w = gsl_vector_alloc(roi.width);
        mw = gsl_multifit_linear_alloc(roi.width, NCOEFFS);
        X = gsl_matrix_alloc(roi.width, NCOEFFS);

        // Assign all weights to be equal
        gsl_vector_set_all(w, 1);

        // Assign the x value
        for (uint32_t i = 0; i < roi.width; i++) gsl_vector_set(x, i, i);

        // use uniform breakpoints on [0, 15]
        gsl_bspline_knots_uniform(0, roi.width, bw);

        double Rsq = 0;
        double chisq;
        double tss;

        // auto start = std::chrono::steady_clock::now();
        // Assign y value
        for (uint32_t i = 0; i < roi.height; ++i) {
            for (uint32_t j = 0; j < roi.width; ++j) {
                uint32_t idx2 = (i + roi.y_offset) * depth_frame.get_width() + (j + roi.x_offset);
                double dis = double(data_ptr[idx2] * 1.0f);
                gsl_vector_set(y, j, dis);

                gsl_bspline_eval(j, B, bw);

                for (uint32_t k = 0; k < NCOEFFS; ++k) {
                    double Bj = gsl_vector_get(B, k);
                    gsl_matrix_set(X, j, k, Bj);
                }

                // gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);
                // tss = gsl_stats_wtss(w->data, 1, y->data, 1, y->size);
                gsl_multifit_linear(X, y, c, cov, &chisq, mw);
                tss = gsl_stats_tss(y->data, 1, y->size);
                Rsq += 1.0 - chisq / tss;
            }

            double yi, yerr;

            for (uint32_t j = 0; j < roi.width; j++) {
                gsl_bspline_eval(j, B, bw);
                gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
                idx = (i + roi.y_offset) * depth_frame.get_width() + (j + roi.x_offset);
                data_ptr[idx] = static_cast<ushort>(yi);
            }
        }

        cv::Mat reconstructed_cropped_filtered_depth_data(cv::Size(depth_frame.get_width(), depth_frame.get_height()), CV_16U,
                                                          (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        reconstructed_cropped_filtered_depth_image_message =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, reconstructed_cropped_filtered_depth_data).toImageMsg();

        reconstructed_cropped_filtered_depth_image_publisher->publish(*reconstructed_cropped_filtered_depth_image_message);

        RCLCPP_INFO(this->get_logger(), "Published");

        auto points = point_cloud.calculate(depth_frame);

        points.export_to_ply("data.ply", color_frame);

        /* point_cloud_message->header = header;
        point_cloud_message->height = depth_frame.get_height();
        point_cloud_message->width = depth_frame.get_width();
        point_cloud_message->point_step = sizeof(rs2::vertex);
        point_cloud_message->row_step = depth_frame.get_width() * point_cloud_message->point_step;
        point_cloud_message->is_dense = true;
        point_cloud_message->data.resize(point_cloud_message->width * point_cloud_message->row_step);

        uint8_t* vertex_ptr = (uint8_t*) points.get_vertices();

        for(uint32_t i = 0; i < point_cloud_message->data.size(); ++i)
        {
            point_cloud_message->data[i] = vertex_ptr[i];
        }

        point_cloud_publisher->publish(*point_cloud_message);
 */
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_raw_depth_imgae_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_filtered_depth_imgae_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_cropped_filtered_depth_image_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
};

int main(int argc, char *argv[])
{
    bw = gsl_bspline_alloc(4, NBREAKS);
    B = gsl_vector_alloc(NCOEFFS);

    c = gsl_vector_alloc(NCOEFFS);
    cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthImagePublisher>());
    rclcpp::shutdown();

    return 0;
}