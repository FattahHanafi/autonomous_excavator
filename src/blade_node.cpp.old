#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class BladePublisher : public rclcpp::Node {
  public:
    explicit BladePublisher() : Node("blade_publisher")
    {
        blade_publisher = this->create_publisher<geometry_msgs::msg::PolygonStamped>("Machine/Blade", 10);
        // timer_ = this->create_wall_timer(10ms, std::bind(&BladePublisher::timer_callback, this));
        joint_state_subscriber =
            this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&BladePublisher::timer_callback, this, _1));
        m_NewFace.header.frame_id = "bucket";
        m_NewFace.polygon.points.reserve(4);
        auto q = geometry_msgs::msg::Point32();
        q.x = 0.0693f;
        q.y = 0.1015f;
        q.z = 0.0187f;
        m_NewFace.polygon.points.push_back(q);
        q.x = 0.3253f;
        q.y = 0.1015f;
        q.z = 0.0187f;
        m_NewFace.polygon.points.push_back(q);
        q.x = 0.3253f;
        q.y = -0.1015f;
        q.z = 0.0187f;
        m_NewFace.polygon.points.push_back(q);
        q.x = 0.0693f;
        q.y = -0.1015f;
        q.z = 0.0187f;
        m_NewFace.polygon.points.push_back(q);
    }

  private:
    void timer_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        m_NewFace.header.stamp = this->get_clock()->now();
        blade_publisher->publish(m_NewFace);
    }
    geometry_msgs::msg::PolygonStamped m_NewFace;

    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr blade_publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BladePublisher>());
    rclcpp::shutdown();

    return 0;
}
