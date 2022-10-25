#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

class JointPiblisher : public rclcpp::Node {
  public:
    JointPiblisher() : Node("joint_publisher")
    {
        joint_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/JointState", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&JointPiblisher::timer_callback, this));
        joint_message.name.resize(4);
        joint_message.position.resize(4, 0.0f);
        joint_message.velocity.resize(4, 0.0f);
        joint_message.effort.resize(4, 0.0f);

        joint_message.name[0] = "Swing";
        joint_message.name[1] = "Boom";
        joint_message.name[2] = "Arm";
        joint_message.name[3] = "Bucket";
    }

  private:
    void timer_callback()
    {
        rclcpp::Time now = this->get_clock()->now();
		joint_message.position[0] = 0.0f;
        joint_message.position[1] = 0.2 * std::sin(1 * now.seconds());
        joint_message.position[2] = 0.3 + 0.2 * std::sin(2 * now.seconds());
        joint_message.position[3] = 0.5 + 0.2 * std::sin(2 * now.seconds());
        joint_message.header.stamp = now;
        joint_publisher->publish(joint_message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_message;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPiblisher>());
    rclcpp::shutdown();
    return 0;
}
