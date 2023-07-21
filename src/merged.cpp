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

#include "base_realsense_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"

#define NCOEFFS 12
#define NBREAKS NCOEFFS - 2

gsl_bspline_workspace *bw;
gsl_vector *B;
gsl_vector *c, *w;
gsl_vector *x, *y;
gsl_matrix *X, *cov;
gsl_multifit_linear_workspace *mw;
double yi, yerr;
double Rsq;
double chisq;
double tss;

using std::placeholders::_1;
using namespace std::chrono_literals;

class DepthImagePublisher : public rclcpp::Node {
 public:
  explicit DepthImagePublisher() : Node("camera_reader") {
    roi_subscriber =
        this->create_subscription<sensor_msgs::msg::RegionOfInterest>("Camera/ROI", 10, std::bind(&DepthImagePublisher::roi_callback, this, _1));

    rgb_publisher = this->create_publisher<sensor_msgs::msg::Image>("Camera/Raw/RGB", 10);
    raw_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("Camera/Raw/Depth", 10);
    filtered_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("Camera/Filtered/Depth", 10);
    cropped_filtered_depth_imgae_publisher = this->create_publisher<sensor_msgs::msg::Image>("Camera/Cropped/Depth", 10);
    reconstructed_cropped_filtered_depth_image_publisher = this->create_publisher<sensor_msgs::msg::Image>("Camera/Reconstructed/Depth", 10);

    rmw_qos_profile_t qos_profile = rmw_qos_profile_system_default;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
    point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("Camera/PointCloud", qos);

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

    m_filters.reserve(4);

    m_header.frame_id = "camera";

    sensor_msgs::msg::PointField p_x, p_y, p_z, p_rgb;
    p_x.name = "x";
    p_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_x.offset = 0;
    p_x.count = 1;

    p_y.name = "y";
    p_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_y.offset = 4;
    p_y.count = 1;

    p_z.name = "z";
    p_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_z.offset = 8;
    p_z.count = 1;

    p_rgb.name = "rgb";
    p_rgb.datatype = sensor_msgs::msg::PointField::UINT32;
    p_rgb.offset = 12;
    p_rgb.count = 1;

    m_pcMessage.fields.push_back(p_x);
    m_pcMessage.fields.push_back(p_y);
    m_pcMessage.fields.push_back(p_z);
    m_pcMessage.fields.push_back(p_rgb);

    rs2::config cfg;
    cfg.enable_all_streams();
    cfg.enable_stream(RS2_STREAM_DEPTH, -1, 1280, 720, RS2_FORMAT_ANY, 5);
    cfg.enable_stream(RS2_STREAM_COLOR, -1, 1280, 720, RS2_FORMAT_ANY, 5);

    m_rsPipe.start(cfg);

    timer = this->create_wall_timer(200ms, std::bind(&DepthImagePublisher::captureImage, this));
  }

 private:
  sensor_msgs::msg::RegionOfInterest m_roi;
  sensor_msgs::msg::Image::SharedPtr m_rgbMessage;
  sensor_msgs::msg::Image::SharedPtr m_rawDepthMessage;
  sensor_msgs::msg::Image::SharedPtr m_filteredDepthMessage;
  sensor_msgs::msg::Image::SharedPtr m_croppedDepthMessage;
  sensor_msgs::msg::Image::SharedPtr m_reconstructedDepthMessage;
  sensor_msgs::msg::PointCloud2 m_pcMessage;

  std_msgs::msg::Header m_header;
  rs2::pipeline m_rsPipe;
  rs2::frameset m_rsFrameset;
  rs2::pointcloud m_rsPointCloud;

  std::vector<rs2::filter> m_filters;
  rs2::decimation_filter m_decFilter;
  rs2::spatial_filter m_spaFilter;
  rs2::temporal_filter m_tmpFilter;
  rs2::hole_filling_filter m_hlfFilter;
  rs2::threshold_filter m_trhFilter;

  void captureImage() {
    m_rsFrameset = m_rsPipe.wait_for_frames();

    rs2::align align(RS2_STREAM_COLOR);
    auto aligned_frames = align.process(m_rsFrameset);

    auto depth_frame = aligned_frames.get_depth_frame();
    auto color_frame = aligned_frames.get_color_frame();

    const uint32_t width = depth_frame.get_width();
    const uint32_t height = depth_frame.get_height();

    m_header.stamp = this->get_clock().get()->now();

    cv::Mat rgb_data(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
    m_rgbMessage = cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::RGB8, rgb_data).toImageMsg();
    rgb_publisher->publish(*m_rgbMessage);

    cv::Mat depth_data(cv::Size(width, height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
    m_rawDepthMessage = cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, depth_data).toImageMsg();
    raw_depth_image_publisher->publish(*m_rawDepthMessage);

    // Set filter values
    int dec_mag = this->get_parameter("decimation_magnitude").get_parameter_value().get<int>();
    dec_mag = (dec_mag > 8) ? 8 : dec_mag;
    dec_mag = (dec_mag < 2) ? 2 : dec_mag;
    m_decFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, dec_mag);

    int spa_mag = this->get_parameter("spatial_magnitude").get_parameter_value().get<int>();
    spa_mag = (spa_mag > 5) ? 5 : spa_mag;
    spa_mag = (spa_mag < 1) ? 1 : spa_mag;
    float spa_sma = this->get_parameter("spatial_smooth_alpha").get_parameter_value().get<float>();
    spa_sma = (spa_sma > 1.00) ? 1.00 : spa_sma;
    spa_sma = (spa_sma < 0.25) ? 0.25 : spa_sma;
    int spa_smd = this->get_parameter("spatial_smooth_delta").get_parameter_value().get<int>();
    spa_smd = (spa_smd > 50) ? 50 : spa_smd;
    spa_smd = (spa_smd < 1) ? 1 : spa_smd;
    // int spa_hlf =
    // this->get_parameter("spatial_hole_filling").get_parameter_value().get<int>();
    // spa_hlf = (spa_hlf > 5) ? 5 : spa_hlf;
    // spa_hlf = (spa_hlf < 0) ? 0 : spa_hlf;
    m_spaFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, spa_mag);
    m_spaFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, spa_sma);
    m_spaFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, spa_smd);

    float tmp_sma = this->get_parameter("temporial_smooth_alpha").get_parameter_value().get<float>();
    tmp_sma = (tmp_sma > 1.00) ? 1.00 : tmp_sma;
    tmp_sma = (tmp_sma < 0.00) ? 0.00 : tmp_sma;
    int tmp_smd = this->get_parameter("temporial_smooth_delta").get_parameter_value().get<int>();
    tmp_smd = (tmp_smd > 20) ? 20 : tmp_smd;
    tmp_smd = (tmp_smd < 1) ? 1 : tmp_smd;
    // int tmp_psi =
    // this->get_parameter("temporial_persistency_index").get_parameter_value().get<int>();
    // tmp_psi = (tmp_psi > 20) ? 20 : tmp_psi;
    // tmp_psi = (tmp_psi < 1) ? 1 : tmp_psi;
    m_tmpFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, tmp_sma);
    m_tmpFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, tmp_smd);

    // int hlf_hlf =
    // this->get_parameter("hole_filling").get_parameter_value().get<int>();
    // hlf_hlf = (hlf_hlf > 2) ? 2 : hlf_hlf;
    // hlf_hlf = (hlf_hlf < 0) ? 0 : hlf_hlf;
    // hlf_filter.set_option(RS2_OPTION_FILTER_OPTION, hlf_hlf);

    float trh_min = this->get_parameter("threshold_min_distance").get_parameter_value().get<float>();
    float trh_max = this->get_parameter("threshold_max_distance").get_parameter_value().get<float>();
    trh_min = (trh_min < 0) ? 0 : trh_min;
    trh_max = (trh_max < trh_min) ? trh_min : trh_max;
    m_trhFilter.set_option(RS2_OPTION_MIN_DISTANCE, trh_min);
    m_trhFilter.set_option(RS2_OPTION_MAX_DISTANCE, trh_max);

    // Declare filters
    m_filters.clear();
    if (this->get_parameter("decimation_filter").get_parameter_value().get<bool>()) m_filters.push_back(m_decFilter);
    if (this->get_parameter("spatial_filter").get_parameter_value().get<bool>()) m_filters.push_back(m_spaFilter);
    if (this->get_parameter("temporial_filter").get_parameter_value().get<bool>()) m_filters.push_back(m_tmpFilter);
    // if
    // (this->get_parameter("hole_filling_filter").get_parameter_value().get<bool>())
    // filters.push_back(hlf_filter);
    if (this->get_parameter("threshold_filter").get_parameter_value().get<bool>()) m_filters.push_back(m_trhFilter);

    for (auto filter : m_filters) depth_frame = depth_frame.apply_filter(filter);

    cv::Mat filtered_depth_data(cv::Size(width, height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    m_filteredDepthMessage = cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, filtered_depth_data).toImageMsg();

    filtered_depth_image_publisher->publish(*m_filteredDepthMessage);

    if (!m_roi.width || !m_roi.height) return;

    uint8_t *data_ptr = (uint8_t *)depth_frame.get_data();

    const uint32_t row_step = width * depth_frame.get_bytes_per_pixel();
    const uint32_t pxl_step = depth_frame.get_bytes_per_pixel();

    for (uint32_t i = 0; i < height; ++i) {
      if (i < m_roi.y_offset || i >= m_roi.y_offset + m_roi.height) {
        memset(&data_ptr[i * row_step], 0, row_step);
      } else {
        memset(&data_ptr[i * row_step], 0, pxl_step * m_roi.x_offset);
        memset(&data_ptr[i * row_step + (m_roi.x_offset + m_roi.width) * pxl_step], 0, row_step - pxl_step * (m_roi.x_offset - m_roi.width));
      }
    }

    cv::Mat cropped_filtered_depth_data(cv::Size(width, height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    m_croppedDepthMessage = cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, cropped_filtered_depth_data).toImageMsg();

    cropped_filtered_depth_imgae_publisher->publish(*m_croppedDepthMessage);

    // Assign y value
    uint32_t idx = 0;
    for (uint32_t i = 0; i < m_roi.height; ++i) {
      for (uint32_t j = 0; j < m_roi.width; ++j) {
        uint32_t idx2 = (i + m_roi.y_offset) * width + (j + m_roi.x_offset);
        uint16_t L = *(uint16_t *)&data_ptr[2 * idx2];
        gsl_vector_set(y, j, double(L));

        gsl_bspline_eval(j, B, bw);
        for (uint32_t k = 0; k < NCOEFFS; ++k) {
          double Bj = gsl_vector_get(B, k);
          gsl_matrix_set(X, j, k, Bj);
        }
      }

      gsl_multifit_linear(X, y, c, cov, &chisq, mw);

      tss = gsl_stats_tss(y->data, 1, y->size);
      Rsq = 1.0 - chisq / tss;

      if (Rsq > 0.90) {
        for (uint32_t j = 0; j < m_roi.width; j++) {
          idx = (i + m_roi.y_offset) * width + (j + m_roi.x_offset);
          gsl_bspline_eval(j, B, bw);
          gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
          uint16_t L = uint16_t(yi);
          // uint16_t L = uint16_t(1000.0f);
          memcpy(&data_ptr[2 * idx], &L, sizeof(uint16_t));
          // data_ptr[idx] = uint16_t(yi);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Approximation failed, Discard data");
        return;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Approximation done, Publish data");

    cv::Mat reconstructed_cropped_filtered_depth_data(cv::Size(width, height), CV_16U, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    m_reconstructedDepthMessage =
        cv_bridge::CvImage(m_header, sensor_msgs::image_encodings::TYPE_16UC1, reconstructed_cropped_filtered_depth_data).toImageMsg();

    reconstructed_cropped_filtered_depth_image_publisher->publish(*m_reconstructedDepthMessage);

    auto points = m_rsPointCloud.calculate(depth_frame);

    m_pcMessage.set__header(m_header);
    m_pcMessage.height = m_roi.height;
    m_pcMessage.width = m_roi.width;
    m_pcMessage.point_step = 3 * sizeof(float) + 1 * sizeof(uint32_t);
    m_pcMessage.row_step = m_roi.width * m_pcMessage.point_step;
    m_pcMessage.is_dense = true;
    m_pcMessage.data.resize(m_pcMessage.height * m_pcMessage.row_step);

    sensor_msgs::PointCloud2Iterator<float> xyz_it(m_pcMessage, "x");
    sensor_msgs::PointCloud2Iterator<uint32_t> rgb_it(m_pcMessage, "rgb");
    const uint8_t *rgb_value = (const uint8_t *)color_frame.get_data();

    const rs2::vertex *point = points.get_vertices();
    for (uint32_t i = m_roi.y_offset; i < m_roi.y_offset + m_roi.height; ++i)
      for (uint32_t j = m_roi.x_offset; j < m_roi.x_offset + m_roi.width; ++j, ++xyz_it, ++rgb_it) {
        uint32_t idx = i * width + j;
        xyz_it[0] = point[idx].x;
        xyz_it[1] = point[idx].y;
        xyz_it[2] = point[idx].z;
        *rgb_it = uint32_t(rgb_value[3 * idx + 0]) << 16;
        *rgb_it += uint32_t(rgb_value[3 * idx + 1]) << 8;
        *rgb_it += uint32_t(rgb_value[3 * idx + 2]) << 0;
      }
    point_cloud_publisher->publish(m_pcMessage);
  }

  void roi_callback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg) {
    m_roi.x_offset = msg->x_offset;
    m_roi.y_offset = msg->y_offset;
    m_roi.width = msg->width;
    m_roi.height = msg->height;
    m_roi.do_rectify = true;

    // Allocate enough memory space
    x = gsl_vector_alloc(m_roi.width);
    y = gsl_vector_alloc(m_roi.width);
    w = gsl_vector_alloc(m_roi.width);
    mw = gsl_multifit_linear_alloc(m_roi.width, NCOEFFS);
    X = gsl_matrix_alloc(m_roi.width, NCOEFFS);

    // Assign all weights to be equal
    gsl_vector_set_all(w, 1);

    // Assign the x value
    for (uint32_t i = 0; i < m_roi.width; ++i) gsl_vector_set(x, i, i);

    // use uniform breakpoints on [0, 15]
    gsl_bspline_knots_uniform(0, m_roi.width, bw);

    RCLCPP_INFO(this->get_logger(), "ROI set = %u, %u, %u, %u", m_roi.x_offset, m_roi.width, m_roi.y_offset, m_roi.height);
  }

  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr roi_subscriber;

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_depth_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr filtered_depth_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_raw_depth_imgae_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cropped_filtered_depth_imgae_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_cropped_filtered_depth_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher;
};

int main(int argc, char *argv[]) {
  bw = gsl_bspline_alloc(4, NBREAKS);
  B = gsl_vector_alloc(NCOEFFS);

  c = gsl_vector_alloc(NCOEFFS);
  cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthImagePublisher>());
  rclcpp::shutdown();

  return 0;
}
