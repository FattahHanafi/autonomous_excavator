#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Largest_empty_iso_rectangle_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Simple_cartesian.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "../include/Blade.h"
#include "../include/MarchingCubes.h"
#include "../include/Vec3.h"

enum TURN {
  RIGHT = 0,
  LEFT = 1,
  NA
};

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

TURN side(const point* a, const point* c, const point* b) {
  double dis = (b->x - a->x) * (c->y - a->y) - (b->y - a->y) * (c->x - a->x);
  return (dis < 0) ? TURN::RIGHT : TURN::LEFT;
};

DIRECTION dir = DIRECTION::EAST;
TURN q = NA;

std::array<int32_t, 9> u;
std::array<int32_t, 9> v;

double Lookup(const double x, const double y, sensor_msgs::msg::PointCloud2* msg) {
  const point p(x, y);
  bool found = false;
  const int32_t x_span = msg->width;
  const int32_t y_span = msg->height;
  double z = 0;

  std::fill_n(u.begin() + 1, u.size() - 1, -1);
  std::fill_n(v.begin() + 1, v.size() - 1, -1);

  sensor_msgs::PointCloud2ConstIterator<double> xyz_it(*msg, "x");
  xyz_it += v[1] * x_span + u[1];
  while (!found) {
    const point p1(xyz_it[0], xyz_it[1]);
    xyz_it += (v[0] - v[1]) * x_span + (u[0] - u[1]);
    const point p0(xyz_it[0], xyz_it[1]);
    z = xyz_it[2];

    switch (dir) {
      case NORTH:
        if (u.at(0) == 0)
          q = LEFT;
        else if (u[0] == (x_span - 1))
          q = RIGHT;
        else
          q = side(&p1, &p0, &p);

        std::copy(u.cbegin() + 1, u.cend(), u.begin());
        std::copy(v.cbegin() + 1, v.cend(), v.begin());
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

        std::copy(u.cbegin() + 1, u.cend(), u.begin());
        std::copy(v.cbegin() + 1, v.cend(), v.begin());
        u.at(0) += (q == RIGHT) ? +1 : -1;
        dir = (q == RIGHT) ? EAST : WEST;
        break;

      case EAST:
        if (v[0] == 0)
          q = RIGHT;
        else if (v.at(0) == (y_span - 1))
          q = LEFT;
        else
          q = side(&p1, &p0, &p);

        std::copy(u.cbegin() + 1, u.cend(), u.begin());
        std::copy(v.cbegin() + 1, v.cend(), v.begin());
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

        std::copy(u.cbegin() + 1, u.cend(), u.begin());
        std::copy(v.cbegin() + 1, v.cend(), v.begin());
        v.at(0) += (q == RIGHT) ? -1 : +1;
        dir = (q == RIGHT) ? SOUTH : NORTH;
        break;
    }

    found = (*u.cbegin() == *u.cend()) && (*v.cbegin() == *v.cend());
  }

  return z;
}

MarchingCubes* mc(

typedef double Number_Type;
typedef CGAL::Simple_cartesian<Number_Type> Kernel;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

using std::placeholders::_1;

class MarchingCubesPublisher : public rclcpp::Node {
 public:
  explicit MarchingCubesPublisher() : Node("marching_cubes_node") {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    m_marker_message.ns = "mc_cubes";
    m_marker_message.id = 0;
    m_marker_message.header.frame_id = "container";
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
    m_marker_message.color.g = 0.8;
    m_marker_message.color.b = 0;
    m_marker_message.color.a = 1;
    m_marker_message.scale.x = 1;
    m_marker_message.scale.y = 1;
    m_marker_message.scale.z = 1;

    m_pc_rotated_message.header.frame_id = "container";
    pc_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>("Camera/PointCloud", 10,
                                                                             std::bind(&MarchingCubesPublisher::pc_callback, this, _1));

    rotated_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("Camera/RotatedPointCloud", 10);
    blade_subscriber =
        this->create_subscription<geometry_msgs::msg::Polygon>("Machine/Blade", 10, std::bind(&MarchingCubesPublisher::blade_callback, this, _1));

    build_subsciber =
        this->create_subscription<std_msgs::msg::Empty>("MarchingCubes/Build", 10, std::bind(&MarchingCubesPublisher::BuildSurface, this, _1));

    update_subsciber =
        this->create_subscription<std_msgs::msg::Empty>("MarchingCubes/Update", 10, std::bind(&MarchingCubesPublisher::UpdateSurface, this, _1));

    marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("MarchingCubes/Cubes", 10);
  }

 private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!m_BuildSurface) return;
    m_BuildSurface = false;
    const auto t = tf_buffer->lookupTransform(m_pc_rotated_message.header.frame_id, msg->header.frame_id, tf2::TimePointZero);

    const uint32_t width = msg->width;
    const uint32_t height = msg->height;
    m_pc_rotated_message.header.stamp = msg->header.stamp;
    m_pc_rotated_message.set__fields(msg->fields);
    m_pc_rotated_message.set__height(height);
    m_pc_rotated_message.set__width(width);
    m_pc_rotated_message.set__is_bigendian(msg->is_bigendian);
    m_pc_rotated_message.set__is_dense(msg->is_dense);
    m_pc_rotated_message.set__point_step(msg->point_step);
    m_pc_rotated_message.set__row_step(msg->row_step);
    m_pc_rotated_message.set__data(msg->data);

    tf2::doTransform(*msg, m_pc_rotated_message, t);

    rotated_pc_publisher->publish(m_pc_rotated_message);

    auto start = std::chrono::steady_clock::now();
    Polygon_2 border;
    sensor_msgs::PointCloud2ConstIterator<double> xyz_it(m_pc_rotated_message, "x");

    while (xyz_it != xyz_it.end()) {
      border.push_back(Point_2(xyz_it[0], xyz_it[1]));
      ++xyz_it;
    }
    auto bbox = border.bbox();

    Iso_rectangle_2 border_box(Point_2(bbox.xmin(), bbox.ymin()), Point_2(bbox.xmax(), bbox.ymax()));
    CGAL::Largest_empty_iso_rectangle_2<Kernel> leir(border_box);

    polygon_type boost_border;
    sensor_msgs::PointCloud2ConstIterator<double> xyz_it2(m_pc_rotated_message, "x");

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
    for (uint32_t i = 1; i < width - 1; ++i) {
      xyz_it2 += -width;
      leir.insert(Point_2(xyz_it2[0], xyz_it2[1]));
      pn.x(xyz_it2[0]);
      pn.y(xyz_it2[1]);
      boost_border.outer().push_back(pn);
    }

    Iso_rectangle_2 lbox = leir.get_largest_empty_iso_rectangle();

    auto end = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(), "creating bbox took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    start = std::chrono::steady_clock::now();

    const double bbxmin = bbox.xmin();
    const double bbxmax = bbox.xmax();
    const double bbymin = bbox.ymin();
    const double bbymax = bbox.ymax();
    const double lbxmin = lbox.xmin();
    const double lbxmax = lbox.xmax();
    const double lbymin = lbox.ymin();
    const double lbymax = lbox.ymax();
    double x;
    double y;
    double zs;

    bool isInside = false;
    for (uint32_t i = 0; i < mc.m_Step.x; ++i) {
      x = i * mc.m_Size.x;
      for (uint32_t j = 0; j < mc.m_Step.y; ++j) {
        y = j * mc.m_Size.y;
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

        zs = isInside ? Lookup(x, y, &m_pc_rotated_message) : 0.0f;
        for (uint32_t k = 0; k < mc.m_Step.z; ++k) {
          mc.SetVertex(i, j, k, (k * mc.m_Size.z) < zs);
        }
      }
    }

    std::mdspan
        end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Generating took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    start = std::chrono::steady_clock::now();
    mc.RebuildCubes();
    Publish();
    end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Rebuild and Publish took %lu ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
  }

  void blade_callback(const geometry_msgs::msg::Polygon::SharedPtr msg) {
    auto t = tf_buffer->lookupTransform("container", "bucket", tf2::TimePointZero);
    m_rotated_blade.points.resize(msg->points.size());

    geometry_msgs::msg::Point p_in;
    geometry_msgs::msg::Point p_out;

    for (uint32_t i = 0; i < msg->points.size(); ++i) {
      p_in.x = msg->points[i].x;
      p_in.y = msg->points[i].y;
      p_in.z = msg->points[i].z;
      tf2::doTransform(p_in, p_out, t);
      m_rotated_blade.points[i].x = p_out.x;
      m_rotated_blade.points[i].y = p_out.y;
      m_rotated_blade.points[i].z = p_out.z;
    }
    mc.SetBlade(&m_rotated_blade);
    mc.CutBlade();
    if (m_UpdateSurface) {
      m_UpdateSurface = false;
      mc.RebuildCubes();
      Publish();
    }

    RCLCPP_INFO(this->get_logger(), "Regenerated");
  }

  void UpdateSurface(const std_msgs::msg::Empty msg) {
    RCLCPP_INFO(this->get_logger(), "Update requested!");
    auto start = std::chrono::steady_clock::now();
    mc.RebuildCubes();
    auto end = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "Rebuilding took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    Publish();
  }

  void BuildSurface(const std_msgs::msg::Empty msg) {
    RCLCPP_INFO(this->get_logger(), "Build requested!");
    m_BuildSurface = true;
  }

  void Publish() {
    m_marker_message.header.stamp = this->get_clock()->now();
    mc.CreateMessage(&m_marker_message);
    marker_publisher->publish(m_marker_message);
  }
  bool m_BuildSurface = false;
  bool m_UpdateSurface = false;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr blade_subscriber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr build_subsciber;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr update_subsciber;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rotated_pc_publisher;

  geometry_msgs::msg::Point32::SharedPtr m_point32;
  geometry_msgs::msg::Polygon::SharedPtr m_polygon_message;
  sensor_msgs::msg::PointCloud2 m_pc_rotated_message;
  visualization_msgs::msg::Marker m_marker_message;
  geometry_msgs::msg::Polygon m_rotated_blade;

  sensor_msgs::msg::PointCloud2 pc_rotated;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
};

int main(int argc, char* argv[]) {
  u.at(0) = 1;
  mc.RebuildCubes();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarchingCubesPublisher>());
  rclcpp::shutdown();

  return 0;
}
