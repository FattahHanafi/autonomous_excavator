#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class HomeStrokePiblisher : public rclcpp::Node {
  public:
    HomeStrokePiblisher() : Node("home_stroke_publisher")
    {
        joint_angle_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Command", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&HomeStrokePiblisher::timer_callback, this));
        stroke_message.name.resize(4);
        stroke_message.position.resize(4, 0.0f);
        stroke_message.velocity.resize(4, 0.0f);
        stroke_message.effort.resize(4, 0.0f);

        stroke_message.name[0] = "S0";
        stroke_message.name[1] = "S1";
        stroke_message.name[2] = "S2";
        stroke_message.name[3] = "S3";
    }

  private:
    void timer_callback()
    {
        auto now = this->get_clock()->now();
        stroke_message.position[0] = 0.0;
        stroke_message.position[1] = 410.0f;
        stroke_message.position[2] = 580.0f;
		stroke_message.position[3] = 410.0f;
        stroke_message.header.stamp = now;
        joint_angle_publisher->publish(stroke_message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState stroke_message;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomeStrokePiblisher>());
    rclcpp::shutdown();
    return 0;
}
