#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class FeedbackLogger : public rclcpp::Node {
  public:
    explicit FeedbackLogger() : Node("feedbacl_logger")
    {
        stroke_feedback_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Feedback", 10,
                                                                                             std::bind(&FeedbackLogger::FeedbackCallback, this, _1));
        statistics_publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("Machine/ActuatorStroke/Statistics", 10);
        for (uint8_t i = 0; i < 16; i++) statistics_message.data.push_back(0);
    }
    uint8_t buffer[16];
    bool isDone = false;
    uint32_t idx = 0;

  private:
    void FeedbackCallback(const sensor_msgs::msg::JointState msg)
    {
        auto it = statistics_message.data.begin();
        for (uint8_t i = 0; i < 4; i++) {
            // it[0]++;
            it[i * 4 + 0] = msg.position[i];
            if (idx == 0) {
                it[i * 4 + 1] = msg.position[i];
                it[i * 4 + 2] = msg.position[i];
                it[i * 4 + 3] = msg.position[i];
                // RCLCPP_INFO(this->get_logger(), "%f", it[4 * i + 0]);
            }
            else {
                it[i * 4 + 1] *= float(idx - 1) / idx;
                it[i * 4 + 1] += msg.position[i] / idx;
                it[i * 4 + 2] = it[i * 4 + 2] < msg.position[i] ? it[i * 4 + 2] : msg.position[i];
                it[i * 4 + 3] = it[i * 4 + 3] > msg.position[i] ? it[i * 4 + 3] : msg.position[i];
            }
        }
        idx++;
        statistics_publisher->publish(statistics_message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr statistics_publisher;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stroke_feedback_subscriber;

    std_msgs::msg::Float32MultiArray statistics_message;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FeedbackLogger>());
    rclcpp::shutdown();
    return 0;
}
