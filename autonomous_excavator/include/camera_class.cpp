#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Largest_empty_iso_rectangle_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Simple_cartesian.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_vector_double.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <MarchingCubes.cpp>
#include <algorithm>
#include <array>
#include <autonomous_excavator_interfaces/action/detail/reconstruct__struct.hpp>
#include <autonomous_excavator_interfaces/action/reconstruct.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <iomanip>
#include <memory>
#include <mutex>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <thread>
#include <unordered_set>
#include <visualization_msgs/msg/detail/marker__struct.hpp>

#define NCOEFFS 12
#define NBREAKS NCOEFFS - 2
#define CORES 24

typedef double Number_Type;
typedef CGAL::Simple_cartesian<Number_Type> Kernel;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

enum TURN { RIGHT = 0, LEFT = 1, NA };

enum DIRECTION {
  NORTH = 0,
  SOUTH,
  EAST,
  WEST,
};

struct point {
  point(double _x, double _y) : x(_x), y(_y){};
  const double x;
  const double y;
};

std::vector<bool> isCurveApproximated;

TURN side(const point *a, const point *b, const point *c) {
  double dis = (b->x - a->x) * (c->y - a->y) - (b->y - a->y) * (c->x - a->x);
  return (dis < 0) ? TURN::RIGHT : TURN::LEFT;
};

DIRECTION dir = DIRECTION::EAST;
TURN q = NA;

std::array<int32_t, 9> u{0};
std::array<int32_t, 9> v{0};

double Lookup(const double x, const double y, sensor_msgs::msg::PointCloud2 *msg) {
  const point p(x, y);
  bool found = false;
  const int32_t x_span = msg->width;
  const int32_t y_span = msg->height;
  std::array<double, 4> z{0};

  std::fill(u.begin() + 2, u.end(), -1);
  std::fill(v.begin() + 2, v.end(), -1);

  sensor_msgs::PointCloud2ConstIterator<float> xyz_it(*msg, "x");
  xyz_it += v.at(1) * x_span + u.at(1);
  while (!found) {
    const point p1((double(xyz_it[0])), (double(xyz_it[1])));
    xyz_it += (v.at(0) - v.at(1)) * x_span + (u.at(0) - u.at(1));
    const point p0((double(xyz_it[0])), (double(xyz_it[1])));
    std::copy_n(z.cbegin(), 3, z.begin() + 1);
    z.at(0) = double(xyz_it[2]);

    switch (dir) {
      case NORTH:
        if (u.at(0) == 0)
          q = LEFT;
        else if (u.at(0) == (x_span - 1))
          q = RIGHT;
        else
          q = side(&p1, &p0, &p);

        // std::copy(u.cbegin(), u.cend(), u.begin() + 1);
        // std::copy(v.cbegin(), v.cend(), v.begin() + 1);
        std::copy_n(u.cbegin(), 8, u.begin() + 1);
        std::copy_n(v.cbegin(), 8, v.begin() + 1);
        u.at(0) += (q == RIGHT) ? -1 : +1;
        dir = (q == RIGHT) ? WEST : EAST;
        break;

      case SOUTH:
        if (u.at(0) == 0)
          q = RIGHT;
        else if (u.at(0) == (x_span - 1))
          q = LEFT;
        else
          q = side(&p1, &p0, &p);

        // std::copy(u.cbegin(), u.cend(), u.begin() + 1);
        // std::copy(v.cbegin(), v.cend(), v.begin() + 1);
        std::copy_n(u.cbegin(), 8, u.begin() + 1);
        std::copy_n(v.cbegin(), 8, v.begin() + 1);
        u.at(0) += (q == RIGHT) ? +1 : -1;
        dir = (q == RIGHT) ? EAST : WEST;
        break;

      case EAST:
        if (v.at(0) == 0)
          q = RIGHT;
        else if (v.at(0) == (y_span - 1))
          q = LEFT;
        else
          q = side(&p1, &p0, &p);

        // std::copy(u.cbegin(), u.cend(), u.begin() + 1);
        // std::copy(v.cbegin(), v.cend(), v.begin() + 1);
        std::copy_n(u.cbegin(), 8, u.begin() + 1);
        std::copy_n(v.cbegin(), 8, v.begin() + 1);
        v.at(0) += (q == RIGHT) ? +1 : -1;
        dir = (q == RIGHT) ? NORTH : SOUTH;
        break;

      case WEST:
        if (v.at(0) == 0)
          q = LEFT;
        else if (v.at(0) == (y_span - 1))
          q = RIGHT;
        else
          q = side(&p1, &p0, &p);

        // std::copy(u.cbegin(), u.cend(), u.begin() + 1);
        // std::copy(v.cbegin(), v.cend(), v.begin() + 1);
        std::copy_n(u.cbegin(), 8, u.begin() + 1);
        std::copy_n(v.cbegin(), 8, v.begin() + 1);
        v.at(0) += (q == RIGHT) ? -1 : +1;
        dir = (q == RIGHT) ? SOUTH : NORTH;
        break;
    }

    found = (u.at(0) == u.at(8)) && (v.at(0) == v.at(8));
  }
  return 0.25 * (z.at(0) + z.at(1) + z.at(2) + z.at(3));
  // return std::accumulate(z.cbegin(), z.cend(), 0) / double(z.size());
};

struct rs2_intrinsics {
  bool isSet = false;
  int width = 0;         /**< Width of the image in pixels */
  int height = 0;        /**< Height of the image in pixels */
  float ppx = 0;         /**< Horizontal coordinate of the principal point of the image, as a pixel offset from the left edge */
  float ppy = 0;         /**< Vertical coordinate of the principal point of the image, as a pixel offset from the top edge */
  float fx = 0;          /**< Focal length of the image plane, as a multiple of pixel width */
  float fy = 0;          /**< Focal length of the image plane, as a multiple of pixel height */
  float coeffs[5] = {0}; /**< Distortion coefficients */
};

class Timer {
 public:
  Timer() { m_start = std::chrono::steady_clock::now(); }

  void reset() { m_start = std::chrono::steady_clock::now(); }

  float time() {
    m_end = std::chrono::steady_clock::now();
    return float(std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count());
  }

 private:
  std::chrono::steady_clock::time_point m_start;
  std::chrono::steady_clock::time_point m_end;
};

Timer timer;

class CameraClass : public rclcpp::Node {
 public:
  using Reconstruct = autonomous_excavator_interfaces::action::Reconstruct;
  using GoalHandleReconstruct = rclcpp_action::ServerGoalHandle<Reconstruct>;
  CameraClass() : Node("surface_reconstructor") {
    u.at(0) = 1;
    v.at(0) = 0;

    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

    m_reconstructServer = rclcpp_action::create_server<Reconstruct>(
        this, this->get_effective_namespace() + "_reconstruct",
        std::bind(&CameraClass::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CameraClass::handle_cancel, this, std::placeholders::_1), std::bind(&CameraClass::handle_accepted, this, std::placeholders::_1));

    m_marchingCubes.ns = this->get_effective_namespace();
    m_marchingCubes.id = 0;
    m_marchingCubes.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    m_marchingCubes.action = visualization_msgs::msg::Marker::ADD;
    m_marchingCubes.pose.position.x = 0;
    m_marchingCubes.pose.position.y = 0;
    m_marchingCubes.pose.position.z = 0;
    m_marchingCubes.pose.orientation.x = 0;
    m_marchingCubes.pose.orientation.y = 0;
    m_marchingCubes.pose.orientation.z = 0;
    m_marchingCubes.pose.orientation.w = 1;
    m_marchingCubes.color.r = 1;
    m_marchingCubes.color.g = 0.8;
    m_marchingCubes.color.b = 0;
    m_marchingCubes.color.a = 1;
    m_marchingCubes.scale.x = 1;
    m_marchingCubes.scale.y = 1;
    m_marchingCubes.scale.z = 1;

    sensor_msgs::msg::PointField p_x, p_y, p_z, p_rgb;
    p_x.name = "x";
    p_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_x.offset = 0 * sizeof(float);
    p_x.count = 1;

    p_y.name = "y";
    p_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_y.offset = 1 * sizeof(float);
    p_y.count = 1;

    p_z.name = "z";
    p_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    p_z.offset = 2 * sizeof(float);
    p_z.count = 1;

    p_rgb.name = "rgb";
    p_rgb.datatype = sensor_msgs::msg::PointField::UINT32;
    p_rgb.offset = 3 * sizeof(float);
    p_rgb.count = 1;

    m_soilPointCloud.fields.push_back(p_x);
    m_soilPointCloud.fields.push_back(p_y);
    m_soilPointCloud.fields.push_back(p_z);
    m_soilPointCloud.fields.push_back(p_rgb);
    m_soilPointCloud.point_step = 3 * sizeof(float) + 1 * sizeof(uint32_t);

    m_buttomSurface.fields.push_back(p_x);
    m_buttomSurface.fields.push_back(p_y);
    m_buttomSurface.fields.push_back(p_z);
    m_buttomSurface.point_step = 3 * sizeof(float);

    m_rotatedButtomSurface.fields = m_buttomSurface.fields;
    m_rotatedButtomSurface.point_step = m_buttomSurface.point_step;

    if (this->get_effective_namespace() == "/Camera_Container") {
      m_MC = std::make_unique<MarchingCubes>(135, 60, 60, 0.010);
      m_marchingCubes.header.frame_id = "container";
      m_depthImage.header.frame_id = "Camera_Container_depth_optical_frame";
      m_reconstructedDepthImage.header.frame_id = "Camera_Container_depth_optical_frame";
      m_soilPointCloud.header.frame_id = "Camera_Container_depth_optical_frame";
      m_reconstructedSoilPointCloud.header.frame_id = "Camera_Container_depth_optical_frame";
      m_buttomSurface.header.frame_id = "container";
      m_buttomSurface.width = 2;
      m_buttomSurface.height = 1;
      m_buttomSurface.row_step = m_buttomSurface.width * m_buttomSurface.point_step;
      m_buttomSurface.data.resize(m_buttomSurface.height * m_buttomSurface.row_step);
      sensor_msgs::PointCloud2Iterator<float> xyz_bt(m_buttomSurface, "x");
      xyz_bt[0] = 0.0f;
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = 0.0f;
      ++xyz_bt;
      xyz_bt[0] = float(m_MC->m_Step.x * m_MC->m_Size.x);
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = 0.0f;
    } else if (this->get_effective_namespace() == "/Camera_Bucket") {
      m_MC = std::make_unique<MarchingCubes>(5.0 * 60, 5.0 * 60, 5.0 * 150, 0.010 / 5.0);
      m_marchingCubes.color.r = 0.5;
      m_marchingCubes.color.g = 0.5;
      m_marchingCubes.color.b = 0.5;
      m_marchingCubes.header.frame_id = "Bucket_container";
      m_depthImage.header.frame_id = "Camera_Bucket_depth_optical_frame";
      m_reconstructedDepthImage.header.frame_id = "Camera_Bucket_depth_optical_frame";
      m_soilPointCloud.header.frame_id = "Camera_Bucket_depth_optical_frame";
      m_reconstructedSoilPointCloud.header.frame_id = "Camera_Bucket_depth_optical_frame";
      m_buttomSurface.header.frame_id = "bucket";
      m_buttomSurface.width = 4;
      m_buttomSurface.height = 1;
      m_buttomSurface.row_step = m_buttomSurface.width * m_buttomSurface.point_step;
      m_buttomSurface.data.resize(m_buttomSurface.height * m_buttomSurface.row_step);
      sensor_msgs::PointCloud2Iterator<float> xyz_bt(m_buttomSurface, "x");
      xyz_bt[0] = 0.34804f;
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = -0.02579f;
      ++xyz_bt;
      xyz_bt[0] = 0.30215f;
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = 0.06393f;
      ++xyz_bt;
      xyz_bt[0] = 0.15408f;
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = 0.17873f;
      ++xyz_bt;
      xyz_bt[0] = 0.06933f;
      xyz_bt[1] = 0.0f;
      xyz_bt[2] = 0.01873f;

    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid name");
      throw "Invalid Namespace";
    }

    bw = gsl_bspline_alloc(4, NBREAKS);
    B = gsl_vector_alloc(NCOEFFS);

    c = gsl_vector_alloc(NCOEFFS);
    cov = gsl_matrix_alloc(NCOEFFS, NCOEFFS);

    m_soilPointCloud.header.frame_id = this->get_effective_namespace() + "_depth_optical_frame";

    m_camInfoSubscriber = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "depth/camera_info", 10, std::bind(&CameraClass::cam_info_callback, this, std::placeholders::_1));
    m_roiSubscriber =
        this->create_subscription<sensor_msgs::msg::RegionOfInterest>("roi", 10, std::bind(&CameraClass::roi_callback, this, std::placeholders::_1));
    m_depthSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "depth/image_rect_raw", 10, std::bind(&CameraClass::depth_image_callback, this, std::placeholders::_1));
    m_colorSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "color/image_raw", 10, std::bind(&CameraClass::color_image_callback, this, std::placeholders::_1));
    m_depthPublisher = this->create_publisher<sensor_msgs::msg::Image>("roi/depth", 10);
    m_colorPublisher = this->create_publisher<sensor_msgs::msg::Image>("roi/color", 10);
    m_soilSurfacePublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("roi/soil_surface", 10);
    m_reconstructedDepthPublisher = this->create_publisher<sensor_msgs::msg::Image>("reconstructed/depth", 10);
    m_reconstructedSoilSurfacePublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("reconstructed/soil_surface", 10);
    m_marchingCubesPublisher = this->create_publisher<visualization_msgs::msg::Marker>("marching_cubes", 10);
    m_testPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("test", 10);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Reconstruct::Goal> goal) {
    (void)&uuid;
    (void)&goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    (void)&goal_handle;
    RCLCPP_INFO(this->get_logger(), "Request Canceled.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    std::thread(std::bind(&CameraClass::execute, this, std::placeholders::_1), goal_handle).detach();
  }

  void buttom_surface_rotate() {
    m_rotatedButtomSurface = sensor_msgs::msg::PointCloud2(m_buttomSurface);
    if (this->get_effective_namespace() == "/Camera_Container")
      m_rotatedButtomSurface.header.frame_id = "container";
    else if (this->get_effective_namespace() == "/Camera_Bucket")
      m_rotatedButtomSurface.header.frame_id = "Bucket_container";

    m_rotatedButtomSurface.header.stamp = rclcpp::Clock().now();
    if (m_buttomSurface.header.frame_id[0] == '/') m_buttomSurface.header.frame_id = m_buttomSurface.header.frame_id.erase(0, 1);

    const auto t = m_tf_buffer->lookupTransform(m_rotatedButtomSurface.header.frame_id, m_buttomSurface.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(m_rotatedButtomSurface, m_buttomSurface, t);
  }

  void execute(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    auto feedback = std::make_shared<Reconstruct::Feedback>();

    if (goal_handle->is_canceling()) return;
    m_reconstructedDepthImage = sensor_msgs::msg::Image(m_depthImage);
    uint16_t *data_ptr = (uint16_t *)m_reconstructedDepthImage.data.data();

    if (data_ptr[0] == 0) data_ptr[0] = 1500;
    for (uint32_t j = 0; j < m_reconstructedDepthImage.height; ++j)
      for (uint32_t i = 0; i < m_reconstructedDepthImage.width; ++i) {
        if (data_ptr[j * m_reconstructedDepthImage.width + i] == 0) {
          data_ptr[j * m_reconstructedDepthImage.width + i] = data_ptr[j * m_reconstructedDepthImage.width + i - 1];
        }
      }
    feedback->process.data = "Depth image copied";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    // CameraClass::curve_approximate(0, m_roi.height);
    for (uint32_t i = 0; i < CORES; ++i)
      std::thread(&CameraClass::curve_approximate, this, i * m_roi.height / CORES, (i + 1) * m_roi.height / CORES).join();
    feedback->process.data = "Curve Approximation done";
    m_reconstructedDepthPublisher->publish(m_reconstructedDepthImage);
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    rs2_deproject_pixel_to_point(&m_reconstructedDepthImage, &m_reconstructedSoilPointCloud);
    feedback->process.data = "Point Cloud generated";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    create_cubes();
    m_reconstructedSoilSurfacePublisher->publish(m_rotatedPointCloud);
    feedback->process.data = "Marching Cube generated";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    m_MC->RebuildCubes();
    double vol = m_MC->CalculateVolume();
    feedback->process.data = "Volume calculated";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    m_MC->CreateMessage(&m_marchingCubes);
    m_marchingCubesPublisher->publish(m_marchingCubes);
    feedback->process.data = "Marching Cubes published";
    goal_handle->publish_feedback(feedback);

    if (goal_handle->is_canceling()) return;
    auto result = std::make_shared<Reconstruct::Result>();
    result->volume = vol * 1000000.0;  // Convert from m^3 to cm^3
    goal_handle->succeed(result);
  }

  void cam_info_callback(const sensor_msgs::msg::CameraInfo &msg) {
    m_intrinsics.width = msg.width;
    m_intrinsics.height = msg.height;
    m_intrinsics.fx = msg.k.at(0);
    m_intrinsics.fy = msg.k.at(4);
    m_intrinsics.ppx = msg.k.at(2);
    m_intrinsics.ppy = msg.k.at(5);
    m_intrinsics.coeffs[0] = msg.d[0];
    m_intrinsics.coeffs[1] = msg.d[1];
    m_intrinsics.coeffs[2] = msg.d[2];
    m_intrinsics.coeffs[3] = msg.d[3];
    m_intrinsics.coeffs[4] = msg.d[4];
    m_intrinsics.isSet = true;
    m_camInfoSubscriber.reset();  // disable this subscriber
  }

  void roi_callback(const sensor_msgs::msg::RegionOfInterest &msg) {
    if ((msg.x_offset + msg.width <= m_depthImage.width) && (msg.y_offset + msg.height <= m_depthImage.height)) {
      m_roi = sensor_msgs::msg::RegionOfInterest(msg);

      m_soilPointCloud.width = m_roi.width;
      m_soilPointCloud.height = m_roi.height;
      m_soilPointCloud.row_step = m_soilPointCloud.width * m_soilPointCloud.point_step;
      m_soilPointCloud.data.resize(m_soilPointCloud.height * m_soilPointCloud.row_step);

      isCurveApproximated.resize(m_roi.height);

      x = gsl_vector_alloc(m_roi.width);
      y = gsl_vector_alloc(m_roi.width);
      mw = gsl_multifit_linear_alloc(m_roi.width, NCOEFFS);
      X = gsl_matrix_alloc(m_roi.width, NCOEFFS);
      for (uint32_t i = 0; i < m_roi.width; ++i) gsl_vector_set(x, i, i);
      gsl_bspline_knots_uniform(0, m_roi.width, bw);
      RCLCPP_INFO(this->get_logger(), "ROI set.");
    } else
      RCLCPP_INFO(this->get_logger(), "Invalid ROI. Skipped");
  }

  void depth_image_callback(const sensor_msgs::msg::Image &msg) {
    m_depthImage = sensor_msgs::msg::Image(msg);

    if (m_roi.width == 0 || m_roi.height == 0) {
      m_depthPublisher->publish(m_depthImage);
      return;
    }

    const uint32_t w = m_depthImage.width;
    const uint32_t h = m_depthImage.height;
    auto ptr = m_depthImage.data.begin();

    for (uint32_t j = 0; j < h; ++j) {
      if (j < m_roi.y_offset || j > m_roi.y_offset + m_roi.height) {
        std::fill(ptr, ptr + 2 * w, 0);
      } else {
        std::fill(ptr, ptr + 2 * m_roi.x_offset, 0);
        std::fill(ptr + 2 * (m_roi.x_offset + m_roi.width), ptr + 2 * w, 0);
      }
      ptr += 2 * w;
    }

    m_depthPublisher->publish(m_depthImage);

    rs2_deproject_pixel_to_point(&m_depthImage, &m_soilPointCloud);
    m_soilSurfacePublisher->publish(m_soilPointCloud);
    buttom_surface_rotate();

    // TEST HERE
    m_rotatedButtomSurface.header.stamp = this->get_clock()->now();
    m_testPublisher->publish(m_rotatedButtomSurface);
  }

  void color_image_callback(const sensor_msgs::msg::Image &msg) {
    m_colorImage = sensor_msgs::msg::Image(msg);

    if (m_roi.width == 0 || m_roi.height == 0) {
      m_colorPublisher->publish(m_colorImage);
      return;
    }

    const uint32_t w = m_colorImage.width;
    const uint32_t h = m_colorImage.height;
    auto ptr = m_colorImage.data.begin();

    for (uint32_t j = 0; j < h; ++j) {
      if (j < m_roi.y_offset || j > m_roi.y_offset + m_roi.height) {
        std::fill(ptr, ptr + 3 * w, 0);
      } else {
        std::fill(ptr, ptr + 3 * m_roi.x_offset, 0);
        std::fill(ptr + 3 * (m_roi.x_offset + m_roi.width), ptr + 3 * w, 0);
      }
      ptr += 3 * w;
    }

    m_colorPublisher->publish(m_colorImage);
  }

  void curve_approximate(uint32_t from, uint32_t to) {
    // m_reconstructedDepthImage = sensor_msgs::msg::Image(m_depthImage);
    m_reconstructedSoilPointCloud = sensor_msgs::msg::PointCloud2(m_soilPointCloud);
    uint16_t *ptr = (uint16_t *)m_reconstructedDepthImage.data.data();
    const uint32_t width = m_reconstructedDepthImage.width;
    const uint32_t w = m_roi.width;
    const uint32_t x_offset = m_roi.x_offset;
    const uint32_t y_offset = m_roi.y_offset;

    for (uint32_t i = 0; i < w; ++i) {
      gsl_bspline_eval(i, B, bw);
      for (uint32_t k = 0; k < NCOEFFS; ++k) {
        double Bj = gsl_vector_get(B, k);
        gsl_matrix_set(X, i, k, Bj);
      }
    }

    for (uint32_t j = from; j < to; ++j) {
      if (isCurveApproximated.at(j) == false) {
        for (uint32_t i = 0; i < w; ++i) {
          gsl_vector_set(y, i, ptr[(j + y_offset) * width + (i + x_offset)]);
        }
        gsl_multifit_linear(X, y, c, cov, &chisq, mw);

        // tss = gsl_stats_tss(y->data, 1, y->size);
        // Rsq = 1.0 - chisq / tss;
        // if (Rsq > 0.15) {
        isCurveApproximated.at(j) = true;
        for (uint32_t i = 0; i < w; ++i) {
          gsl_bspline_eval(i, B, bw);
          gsl_multifit_linear_est(B, c, cov, &yi, &yerr);
          ptr[(j + y_offset) * width + (i + x_offset)] = yi;
        }
        // }
      }
    }
  }

  void rs2_deproject_pixel_to_point(sensor_msgs::msg::Image *depth, sensor_msgs::msg::PointCloud2 *pc) {
    const uint32_t w = depth->width;
    sensor_msgs::PointCloud2Iterator<float> xyz_it(*pc, "x");
    sensor_msgs::PointCloud2Iterator<uint32_t> rgb_it(*pc, "rgb");

    float x, y, r2, f, ux, uy, distance;
    uint16_t *depth_ptr = (uint16_t *)depth->data.data();
    uint8_t *color_ptr = (uint8_t *)m_colorImage.data.data();
    for (uint32_t j = m_roi.y_offset; j < m_roi.y_offset + m_roi.height; ++j) {
      y = (j - m_intrinsics.ppy) / m_intrinsics.fy;
      for (uint32_t i = m_roi.x_offset; i < m_roi.x_offset + m_roi.width; ++i) {
        x = (i - m_intrinsics.ppx) / m_intrinsics.fx;
        r2 = x * x + y * y;
        f = 1 + m_intrinsics.coeffs[0] * r2 + m_intrinsics.coeffs[1] * r2 * r2 + m_intrinsics.coeffs[4] * r2 * r2 * r2;
        ux = x * f + 2 * m_intrinsics.coeffs[2] * x * y + m_intrinsics.coeffs[3] * (r2 + 2 * x * x);
        uy = y * f + 2 * m_intrinsics.coeffs[3] * x * y + m_intrinsics.coeffs[2] * (r2 + 2 * y * y);
        distance = 0.001f * depth_ptr[j * w + i];  // mm to m
        xyz_it[0] = distance * ux;
        xyz_it[1] = distance * uy;
        xyz_it[2] = distance;
        rgb_it[0] = color_ptr[3 * (j * w + i) + 0] << 16;
        rgb_it[0] += color_ptr[3 * (j * w + i) + 1] << 8;
        rgb_it[0] += color_ptr[3 * (j * w + i) + 2] << 0;
        ++xyz_it;
        ++rgb_it;
      }
    }
  }

  void create_cubes() {
    const uint32_t width = m_reconstructedSoilPointCloud.width;
    const uint32_t height = m_reconstructedSoilPointCloud.height;
    m_rotatedPointCloud = sensor_msgs::msg::PointCloud2(m_reconstructedSoilPointCloud);
    if (this->get_effective_namespace() == "/Camera_Container")
      m_rotatedPointCloud.header.frame_id = "container";
    else if (this->get_effective_namespace() == "/Camera_Bucket")
      m_rotatedPointCloud.header.frame_id = "Bucket_container";

    if (m_reconstructedSoilPointCloud.header.frame_id[0] == '/')
      m_reconstructedSoilPointCloud.header.frame_id = m_reconstructedSoilPointCloud.header.frame_id.erase(0, 1);
    m_rotatedPointCloud.header.stamp = rclcpp::Clock().now();

    const auto t =
        m_tf_buffer->lookupTransform(m_rotatedPointCloud.header.frame_id, m_reconstructedSoilPointCloud.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(m_reconstructedSoilPointCloud, m_rotatedPointCloud, t);
    Polygon_2 border;
    sensor_msgs::PointCloud2ConstIterator<float> xyz_it(m_rotatedPointCloud, "x");

    while (xyz_it != xyz_it.end()) {
      border.push_back(Point_2(xyz_it[0], xyz_it[1]));
      border.push_back(Point_2(xyz_it[0], xyz_it[1]));
      ++xyz_it;
    }
    auto bbox = border.bbox();

    Iso_rectangle_2 border_box(Point_2(bbox.xmin(), bbox.ymin()), Point_2(bbox.xmax(), bbox.ymax()));
    CGAL::Largest_empty_iso_rectangle_2<Kernel> leir(border_box);

    const double bbxmin = bbox.xmin();
    const double bbxmax = bbox.xmax();
    const double bbymin = bbox.ymin();
    const double bbymax = bbox.ymax();

    polygon_type boost_border;
    sensor_msgs::PointCloud2ConstIterator<float> xyz_it2(m_rotatedPointCloud, "x");

    point_type pn;
    for (uint32_t i = 0; i < width; ++i) {
      leir.insert(Point_2(xyz_it2[0], xyz_it2[1]));
      pn.x(xyz_it2[0]);
      pn.y(xyz_it2[1]);
      boost_border.outer().push_back(pn);
      ++xyz_it2;
    }

    xyz_it2 += -1;
    for (uint32_t i = 1; i < height; ++i) {
      xyz_it2 += width;
      leir.insert(Point_2(xyz_it2[0], xyz_it2[1]));
      pn.x(xyz_it2[0]);
      pn.y(xyz_it2[1]);
      boost_border.outer().push_back(pn);
    }
    for (uint32_t i = 1; i < width; ++i) {
      xyz_it2 += -1;
      leir.insert(Point_2(xyz_it2[0], xyz_it2[1]));
      pn.x(xyz_it2[0]);
      pn.y(xyz_it2[1]);
      boost_border.outer().push_back(pn);
    }
    for (uint32_t i = 1; i < height; ++i) {
      xyz_it2 += -width;
      leir.insert(Point_2(xyz_it2[0], xyz_it2[1]));
      pn.x(xyz_it2[0]);
      pn.y(xyz_it2[1]);
      boost_border.outer().push_back(pn);
    }

    Iso_rectangle_2 lbox = leir.get_largest_empty_iso_rectangle();

    const double lbxmin = lbox.xmin();
    const double lbxmax = lbox.xmax();
    const double lbymin = lbox.ymin();
    const double lbymax = lbox.ymax();

    double x;
    double y;
    double z;
    double zs;
    double zb;

    bool isInside = false;

    for (uint32_t i = 0; i < m_MC->m_Step.x + 1; ++i) {
      x = m_MC->m_Orig.x + i * m_MC->m_Size.x;
      for (uint32_t j = 0; j < m_MC->m_Step.y + 1; ++j) {
        y = m_MC->m_Orig.y + j * m_MC->m_Size.y;
        if (x < bbxmin || x > bbxmax || y < bbymin || y > bbymax) {
          isInside = false;
        } else if (x >= lbxmin && x <= lbxmax && y >= lbymin && y <= lbymax) {
          isInside = true;
        } else {
          point_type pn;
          pn.x(x);
          pn.y(y);
          isInside = boost::geometry::within(pn, boost_border);
        }
        zs = isInside ? Lookup(x, y, &m_rotatedPointCloud) : 0.0f;

        sensor_msgs::PointCloud2ConstIterator<float> xyz_bt(m_rotatedButtomSurface, "x");

        bool found;

        do {
          float x0 = xyz_bt[0];
          float z0 = xyz_bt[2];
          ++xyz_bt;
          float x1 = xyz_bt[0];
          float z1 = xyz_bt[2];
          found = x >= x0 && x <= x1;
          if (found) {
            zb = double((float(x) - x0) / (x1 - x0) * (z1 - z0) + z0);
            // RCLCPP_INFO(this->get_logger(), "D = %f . %f , %f, %f, %f, %f, %g", x0, z0, float(x), float(zb), x1, z1, float(zs));
          }

        } while (!found);

        // zb = 0;
        // zs = 1.5;
        for (uint32_t k = 0; k < m_MC->m_Step.z + 1; ++k) {
          z = k * m_MC->m_Size.z;
          m_MC->SetVertex(i, j, k, z > zb && z <= zs);
        }
      }
    }
  }

 private:
  rclcpp_action::Server<autonomous_excavator_interfaces::action::Reconstruct>::SharedPtr m_reconstructServer;

  sensor_msgs::msg::RegionOfInterest m_roi;
  sensor_msgs::msg::Image m_depthImage;
  sensor_msgs::msg::Image m_colorImage;
  sensor_msgs::msg::PointCloud2 m_soilPointCloud;
  sensor_msgs::msg::Image m_reconstructedDepthImage;
  sensor_msgs::msg::PointCloud2 m_reconstructedSoilPointCloud;
  sensor_msgs::msg::PointCloud2 m_rotatedPointCloud;
  visualization_msgs::msg::Marker m_marchingCubes;
  sensor_msgs::msg::PointCloud2 m_buttomSurface;
  sensor_msgs::msg::PointCloud2 m_rotatedButtomSurface;

  rs2_intrinsics m_intrinsics;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_camInfoSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr m_roiSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_depthSubscriber;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_colorSubscriber;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depthPublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_colorPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_soilSurfacePublisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_reconstructedDepthPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_reconstructedSoilSurfacePublisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_marchingCubesPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_testPublisher;

  std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  gsl_bspline_workspace *bw;
  gsl_vector *B, *c, *x, *y;
  gsl_matrix *X, *cov;
  gsl_multifit_linear_workspace *mw;
  double yi, yerr;
  // double Rsq;
  double chisq;
  // double tss;

  std::unique_ptr<MarchingCubes> m_MC;
};
