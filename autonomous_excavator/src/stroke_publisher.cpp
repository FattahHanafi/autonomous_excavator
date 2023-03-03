#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class StrokePiblisher : public rclcpp::Node {
 public:
  StrokePiblisher() : Node("stroke_publisher") {
    this->declare_parameter("depth", 530.0f);
    joint_angle_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Command", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&StrokePiblisher::timer_callback, this));
    stroke_message.name.resize(4);
    stroke_message.position.resize(4, 0.0f);
    stroke_message.velocity.resize(4, 0.0f);
    stroke_message.effort.resize(4, 0.0f);

    stroke_message.name[0] = "S0";
    stroke_message.name[1] = "S1";
    stroke_message.name[2] = "S2";
    stroke_message.name[3] = "S3";

    start_time = this->get_clock()->now();
    step_time = 5.0f;
  }

 private:
  void timer_callback() {
    int n;
    std::cout << "Select pose number (1 - 7) = ";
    std::cin >> n;
    float depth = this->get_parameter("depth").get_parameter_value().get<float>();

    switch (n) {
      case 1:
        stroke_message.position[0] = 0.0f;
        stroke_message.position[1] = 430.0f;
        stroke_message.position[2] = 580.0f;
        stroke_message.position[3] = 420.0f;
        break;
      case 2:
        stroke_message.position[0] = 0.0f;
        stroke_message.position[1] = depth;
        stroke_message.position[2] = 580.0f;
        stroke_message.position[3] = 420.0f;
        break;
      case 3:
        stroke_message.position[0] = 0.0f;
        stroke_message.position[1] = depth;
        stroke_message.position[2] = 450.0f;
        stroke_message.position[3] = 580.0f;
        break;
      case 4:
        stroke_message.position[0] = 0.0f;
        stroke_message.position[1] = 430.0f;
        stroke_message.position[2] = 450.0f;
        stroke_message.position[3] = 580.0f;
        break;
      case 5:
        stroke_message.position[0] = 30.0f;
        stroke_message.position[1] = 430.0f;
        stroke_message.position[2] = 450.0f;
        stroke_message.position[3] = 580.0f;
        break;
      case 6:
        stroke_message.position[0] = 30.0f;
        stroke_message.position[1] = 430.0f;
        stroke_message.position[2] = 580.0f;
        stroke_message.position[3] = 420.0f;
        break;
    }
    if (n >= 1 && n <= 6) {
      stroke_message.header.stamp = this->get_clock()->now();
      joint_angle_publisher->publish(stroke_message);
    }
  }

  rclcpp::Time start_time;
  float step_time;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState stroke_message;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_publisher;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StrokePiblisher>());
  rclcpp::shutdown();
  return 0;
}
