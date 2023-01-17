#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SerialInterface : public rclcpp::Node {
  public:
    explicit SerialInterface() : Node("serial_interface")
    {
        m_FeedbackMessage.header.frame_id = "world";
        m_FeedbackMessage.name.push_back("S0");
        m_FeedbackMessage.name.push_back("S1");
        m_FeedbackMessage.name.push_back("S2");
        m_FeedbackMessage.name.push_back("S3");

        m_FeedbackMessage.position.resize(4, 0.0f);
        m_FeedbackMessage.velocity.resize(4, 0.0f);
        m_FeedbackMessage.effort.resize(4, 0.0f);

        serial_rx_subscriber =
            this->create_subscription<std_msgs::msg::UInt8MultiArray>("serial_read", 10, std::bind(&SerialInterface::FeedbackCallback, this, _1));
        stroke_feedback_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Feedback", 10);

        stroke_command_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Command", 10,
                                                                                            std::bind(&SerialInterface::CommandCallback, this, _1));
        serial_tx_publisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>("serial_write", 10);
    }
    uint8_t idx = 0;
    uint8_t buffer[16];
    bool isDone = false;

  private:
    void FeedbackCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
    {
        for (unsigned long it = 0; it < msg->data.size(); ++it) {
            buffer[idx] = msg->data[it];
            if (buffer[idx] == '\n') {
                isDone = true;
                idx = 0;
            }
            else {
                idx++;
            }
        }

        if (isDone) {
            isDone = false;
            float f[4];
            memcpy(f, buffer, sizeof(f));
            m_FeedbackMessage.position[0] = 0.0 * double(f[0]);
            m_FeedbackMessage.position[1] = double(f[1]);
            m_FeedbackMessage.position[2] = double(f[2]);
            m_FeedbackMessage.position[3] = double(f[3]);
            m_FeedbackMessage.header.stamp = this->get_clock()->now();
            stroke_feedback_publisher->publish(m_FeedbackMessage);
        }
    }

    void CommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        m_tx_message.data.resize(4 * sizeof(float) + 1);
		float f[4];
		f[0] = float(msg->position[0]);
		f[1] = float(msg->position[1]);
		f[2] = float(msg->position[2]);
		f[3] = float(msg->position[3]);
		memcpy(&m_tx_message.data[0], f, sizeof(f));
        m_tx_message.data[16] = '\n' - 0;
        serial_tx_publisher->publish(m_tx_message);
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr stroke_feedback_publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stroke_command_subscriber;

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_tx_publisher;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr serial_rx_subscriber;
    sensor_msgs::msg::JointState m_FeedbackMessage;
    std_msgs::msg::UInt8MultiArray m_rx_message;
    std_msgs::msg::UInt8MultiArray m_tx_message;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}
