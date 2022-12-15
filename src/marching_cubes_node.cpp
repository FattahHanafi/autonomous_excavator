#include <CGAL/Iso_rectangle_2.h>
#include <CGAL/Largest_empty_iso_rectangle_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Simple_cartesian.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <bitset>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>
#include <geometry_msgs/msg/point32.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "../include/Blade.h"
#include "../include/MarchingCubes.h"
#include "../include/Vec3.h"

MarchingCubes mc = MarchingCubes(135.0f, 60.0f, 60.0f, 0.01f, 0.01f, 0.01f);

typedef float Number_Type;
typedef CGAL::Simple_cartesian<Number_Type> Kernel;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Iso_rectangle_2 Iso_rectangle_2;

typedef boost::geometry::model::d2::point_xy<float> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

using std::placeholders::_1;

class MarchingCubesPublisher : public rclcpp::Node {
  public:
    explicit MarchingCubesPublisher() : Node("marching_cubes_node")
    {
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        m_marker_message.ns = "mc_cubes";
        m_marker_message.id = 0;
        m_marker_message.header.frame_id = "container";
        m_marker_message.type = 11;
        m_marker_message.action = 0;
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
        blade_subscriber =
            this->create_subscription<geometry_msgs::msg::Polygon>("Machine/Blade", 10, std::bind(&MarchingCubesPublisher::blade_callback, this, _1));

        build_subsciber =
            this->create_subscription<std_msgs::msg::Empty>("MarchingCubes/Build", 10, std::bind(&MarchingCubesPublisher::BuildSurface, this, _1));

        update_subsciber =
            this->create_subscription<std_msgs::msg::Empty>("MarchingCubes/Update", 10, std::bind(&MarchingCubesPublisher::UpdateSurface, this, _1));

        marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("MarchingCubes/Cubes", 10);
    }

  private:
    void pc_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!m_BuildSurface) return;
        m_BuildSurface = false;
        auto t = tf_buffer->lookupTransform(m_pc_rotated_message.header.frame_id, msg->header.frame_id, tf2::TimePointZero);

        m_pc_rotated_message.header.stamp = msg->header.stamp;
        m_pc_rotated_message.set__fields(msg->fields);
        m_pc_rotated_message.set__height(msg->height);
        m_pc_rotated_message.set__width(msg->width);
        m_pc_rotated_message.set__is_bigendian(msg->is_bigendian);
        m_pc_rotated_message.set__is_dense(msg->is_dense);
        m_pc_rotated_message.set__point_step(msg->point_step);
        m_pc_rotated_message.set__row_step(msg->row_step);
        m_pc_rotated_message.set__data(msg->data);

        geometry_msgs::msg::Point p_in;
        geometry_msgs::msg::Point p_out;

        sensor_msgs::PointCloud2Iterator<float> xyz_it(*msg, "x");
        auto start = std::chrono::steady_clock::now();
        while (xyz_it != xyz_it.end()) {
            p_in.x = xyz_it[0];
            p_in.y = xyz_it[1];
            p_in.z = xyz_it[2];
            tf2::doTransform(p_in, p_out, t);
            xyz_it[0] = float(p_out.x);
            xyz_it[1] = float(p_out.y);
            xyz_it[2] = float(p_out.z);
            ++xyz_it;
        }
        auto end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Rotation took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        start = std::chrono::steady_clock::now();
        Polygon_2 border;
        sensor_msgs::PointCloud2Iterator<float> xyz_it2(*msg, "x");

        while (xyz_it2 != xyz_it2.end()) {
            border.push_back(Point_2(xyz_it2[0], xyz_it2[1]));
            ++xyz_it2;
        }
        auto bbox = border.bbox();

        Iso_rectangle_2 border_box(Point_2(bbox.xmin(), bbox.ymin()), Point_2(bbox.xmax(), bbox.ymax()));
        CGAL::Largest_empty_iso_rectangle_2<Kernel> leir(border_box);

        polygon_type boost_border;
        sensor_msgs::PointCloud2Iterator<float> xyz_it3(*msg, "x");
        point_type poin;
        for (uint32_t i = 0; i < msg->width; ++i) {
            leir.insert(Point_2(xyz_it3[0], xyz_it3[1]));
            poin.x(xyz_it3[0]);
            poin.y(xyz_it3[1]);
            boost_border.outer().push_back(poin);
            ++xyz_it3;
        }

        xyz_it3 += -1;

        for (uint32_t i = 1; i < msg->height; ++i) {
            xyz_it3 += msg->width;
            leir.insert(Point_2(xyz_it3[0], xyz_it3[1]));
            poin.x(xyz_it3[0]);
            poin.y(xyz_it3[1]);
            boost_border.outer().push_back(poin);
        }
        for (uint32_t i = 1; i < msg->width; ++i) {
            xyz_it3 += -1;
            leir.insert(Point_2(xyz_it3[0], xyz_it3[1]));
            poin.x(xyz_it3[0]);
            poin.y(xyz_it3[1]);
            boost_border.outer().push_back(poin);
        }
        for (uint32_t i = 1; i < msg->width - 1; ++i) {
            xyz_it3 += -msg->width;
            leir.insert(Point_2(xyz_it3[0], xyz_it3[1]));
            poin.x(xyz_it3[0]);
            poin.y(xyz_it3[1]);
            boost_border.outer().push_back(poin);
        }

        Iso_rectangle_2 lbox = leir.get_largest_empty_iso_rectangle();

        end = std::chrono::steady_clock::now();

        RCLCPP_INFO(this->get_logger(), "creating bbox took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        RCLCPP_INFO(this->get_logger(), "bbox = (%f,%f) , (%f,%f) = %f", bbox.xmin(), bbox.ymin(), bbox.xmax(), bbox.ymax(),
                    bbox.x_span() * bbox.y_span());
        RCLCPP_INFO(this->get_logger(), "leir = (%f,%f) , (%f,%f) = %f", lbox.xmin(), lbox.ymin(), lbox.xmax(), lbox.ymax(), lbox.area());

        start = std::chrono::steady_clock::now();
        uint32_t outside = 0;
        uint32_t inside = 0;
        uint32_t maybe = 0;
        uint32_t total_maybe = 0;

        const float bbxmin = bbox.xmin();
        const float bbxmax = bbox.xmax();
        const float bbymin = bbox.ymin();
        const float bbymax = bbox.ymax();
        const float lbxmin = lbox.xmin();
        const float lbxmax = lbox.xmax();
        const float lbymin = lbox.ymin();
        const float lbymax = lbox.ymax();
        float x;
        float y;
        float zs;
        float z = 0;

        const auto cbegin = border.vertices_begin();
        const auto cend = border.vertices_end();
        const auto KK = Kernel();
        for (uint32_t i = 0; i < mc.m_Step.x; ++i) {
            x = i * mc.m_Size.x;
            for (uint32_t j = 0; j < mc.m_Step.y; ++j) {
                y = j * mc.m_Size.y;
                if (x < bbxmin || x > bbxmax || y < bbymin || y > bbymax) {
                    zs = 0.1f;
                    outside++;
                }
                else if (x >= lbxmin && x <= lbxmax && y >= lbymin && y <= lbymax) {
                    zs = 0.25f;
                    inside++;
                    // evaluate zs by edge hopping
                }
                else {
                    maybe++;
                    point_type poin;
                    poin.x(x);
                    poin.y(y);
                    if (boost::geometry::within(poin, boost_border)) {
                        zs = 0.7f;
                    }
                    else {
                        zs = 0.1f;
                    }
                }

                for (uint32_t k = 0; k < mc.m_Step.z; ++k) {
					mc.SetVertex(i, j, k, (k * mc.m_Size.z) <= zs);
                }
            }
        }
        end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Boost checing box took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        start = std::chrono::steady_clock::now();
		mc.RebuildCubes();
		Publish();
        end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Rebuild and Publish took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    }

    void blade_callback(const geometry_msgs::msg::Polygon::SharedPtr msg)
    {
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
    }

    void UpdateSurface(const std_msgs::msg::Empty msg)
    {
        RCLCPP_INFO(this->get_logger(), "Update requested!");
        auto start = std::chrono::steady_clock::now();
        mc.RebuildCubes();
        auto end = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "Rebuilding took %lu ms", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        Publish();
    }

    void BuildSurface(const std_msgs::msg::Empty msg)
    {
        RCLCPP_INFO(this->get_logger(), "Build requested!");
        m_BuildSurface = true;
    }

    void Publish()
    {
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

    geometry_msgs::msg::Point32::SharedPtr m_point32;
    geometry_msgs::msg::Polygon::SharedPtr m_polygon_message;
    sensor_msgs::msg::PointCloud2 m_pc_rotated_message;
    visualization_msgs::msg::Marker m_marker_message;
    geometry_msgs::msg::Polygon m_rotated_blade;

    sensor_msgs::msg::PointCloud2 pc_rotated;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
};

int main(int argc, char* argv[])
{
    mc.RebuildCubes();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MarchingCubesPublisher>());
    rclcpp::shutdown();

    return 0;
}

bool LineSide(Vec3* a, Vec3* b, Vec3* p)
{
    Vec3 pa = *p - *a;
    Vec3 pb = *p - *a;
    return (pa.Dot2D(&pb) > 0);
};

std::bitset<4> RectangleSide(Vec3* bottom_left, Vec3* bottom_right, Vec3* top_right, Vec3* top_left, Vec3* point)
{
    std::bitset<4> sides;
    sides[0] = LineSide(bottom_left, bottom_right, point);
    sides[1] = LineSide(bottom_right, top_right, point);
    sides[2] = LineSide(top_right, top_left, point);
    sides[3] = LineSide(top_left, bottom_left, point);
    return sides;
};

