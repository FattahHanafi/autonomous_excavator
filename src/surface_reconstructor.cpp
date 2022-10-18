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
            "camera/cropped_raw_depth_image", 10, std::bind(&SurfaceReconstructor::reconstruction_callback, this, _1));
    }

  private:
    std_msgs::msg::Header header = std_msgs::msg::Header();
    sensor_msgs::msg::Image::SharedPtr reconstructed_depth_image_message;

    void reconstruction_callback(sensor_msgs::msg::Image::SharedPtr msg)
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

        auto start = std::chrono::steady_clock::now();
        // Assign y value
        for (uint32_t i = 0; i < msg->height; i++) {
            for (uint32_t j = 0; j < msg->width; j++) {
                double dis = double(cv_ptr->image.at<ushort>(i, j));
                gsl_vector_set(y, j, dis * 0.001);

                gsl_bspline_eval(j, B, bw);

                for (uint32_t k = 0; k < NCOEFFS; ++k) {
                    double Bj = gsl_vector_get(B, k);
                    gsl_matrix_set(X, j, k, Bj);
                }

				gsl_multifit_wlinear(X, w, y, c, cov, &chisq, mw);
                // tss = gsl_stats_wtss(w->data, 1, y->data, 1, y->size);
                // Rsq += 1.0 - chisq / tss;
            }
        }
        auto end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Elapsed time in microseconds: %ld µs",
                    std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());
        // RCLCPP_INFO(this->get_logger(), "Average RSQ = %f", Rsq);

        // raw_depth_imgae_publisher->publish(*depth_image_message);
        // filtered_depth_imgae_publisher->publish(*depth_image_message);
        // filtered_reconstructed_depth_imgae_publisher->publish(*depth_image_message);
        // reconstructed_filtered_depth_imgae_publisher->publish(*depth_image_message);
        // reconstructed_depth_imgae_publisher->publish(*roi_depth_image_message);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr reconstructed_raw_depth_image_publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cropped_raw_depth_image_subscriber;
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
