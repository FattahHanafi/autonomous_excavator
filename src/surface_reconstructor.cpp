#include <cv_bridge/cv_bridge.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_vector_long_double.h>
#include <math.h>
#include <sys/types.h>

#include <chrono>
#include <cstdint>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#define NCOEFFS 12
#define NBREAKS NCOEFFS - 2

gsl_bspline_workspace *bw;
gsl_vector *B;
gsl_vector *c, *w;
gsl_vector *x, *y;
gsl_matrix *X, *cov;
gsl_multifit_linear_workspace *mw;

using std::placeholders::_1;

class SurfaceReconstructor : public rclcpp::Node {
  public:
    explicit SurfaceReconstructor() : Node("surface_reconstrcutor")
    {
        header.frame_id = "camera_depth_optical_frame";
        cropped_raw_depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/cropped_raw_depth_image", 10, std::bind(&SurfaceReconstructor::reconstruction_raw_callback, this, _1));
        reconstructed_cropped_raw_depth_image_publisher =
            this->create_publisher<sensor_msgs::msg::Image>("camera/reconstructed_cropped_raw_depth_image", 10);

        /* cropped_filtered_depth_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/cropped_raw_depth_image", 10, std::bind(&SurfaceReconstructor::reconstruction_filtered_callback, this, _1));
        reconstructed_cropped_filtered_depth_image_publisher =
            this->create_publisher<sensor_msgs::msg::Image>("camera/reconstructed_cropped_filtered_depth_image"); */
    }

  private:
    std_msgs::msg::Header header = std_msgs::msg::Header();
    sensor_msgs::msg::Image::SharedPtr reconstructed_cropped_raw_depth_image_message;
    sensor_msgs::msg::Image::SharedPtr reconstructed_cropped_filtered_depth_image_message;

    void reconstruction_raw_callback(sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        // Allocate enough memory space
        x = gsl_vector_alloc(msg->width);
        y = gsl_vector_alloc(msg->width);
        w = gsl_vector_alloc(msg->width);
        mw = gsl_multifit_linear_alloc(msg->width, NCOEFFS);
        X = gsl_matrix_alloc(msg->width, NCOEFFS);

        // Assign all weights to be equal
        gsl_vector_set_all(w, 1);

        // Assign the x value
        for (uint32_t i = 0; i < msg->width; i++) gsl_vector_set(x, i, i);

        // use uniform breakpoints on [0, 15]
        gsl_bspline_knots_uniform(0, msg->width, bw);

        double Rsq = 0;
        double chisq;
        double tss;

        // auto start = std::chrono::steady_clock::now();
        // Assign y value
        for (uint32_t i = 0; i < msg->height; i++) {
            for (uint32_t j = 0; j < msg->width; j++) {
                double dis = double(cv_ptr->image.at<ushort>(j, i));
                gsl_vector_set(y, j, dis * 0.001);

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

            for (uint32_t j = 0; j < msg->width; j++) {
                gsl_bspline_eval(j, B, bw);
                gsl_multifit_linear_est(B, c, cov, &yi, &yerr);

                cv_ptr->image.at<ushort>(j, i) = ushort(yi / 0.001);
            }
        }

        header.stamp = this->get_clock().get()->now();

        reconstructed_cropped_raw_depth_image_message =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, cv_ptr->image).toImageMsg();
        reconstructed_cropped_raw_depth_image_publisher->publish(*reconstructed_cropped_raw_depth_image_message);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_raw_depth_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_cropped_raw_depth_image_publisher;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_filtered_depth_image_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_cropped_filterted_depth_image_publisher;
};

int main(int argc, char *argv[])
{
    bw = gsl_bspline_alloc(4, NBREAKS);
    B = gsl_vector_alloc(NCOEFFS);

    c = gsl_vector_alloc(NCOEFFS);
    cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SurfaceReconstructor>());
    rclcpp::shutdown();

    return 0;
}
