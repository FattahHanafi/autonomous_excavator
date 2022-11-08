#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class SerialRX : public rclcpp::Node {
  public:
    explicit SerialRX() : Node("serial_rx")
    {
        std::string m_FeedbackSerialPort = "/dev/ttyACM";

        std::string cmd = "ls -l /dev/serial/by-id";
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }

        const std::regex ard_regex("usb-Arduino__www.arduino.cc__");
        const std::regex ser_regex("[0-9A-Z]{20}");
        const std::regex acm_regex("[0-9]$");

        std::stringstream ss_results(result);
        std::string line;
        std::smatch ard_match;
        std::smatch ser_match;
        std::smatch acm_match;
        while (std::getline(ss_results, line, '\n')) {
            if (std::regex_search(line, ard_match, ard_regex)) {
                std::regex_search(line, ser_match, ser_regex);
                std::regex_search(line, acm_match, acm_regex);
                if (ser_match.str() == "75130303236351612250") m_FeedbackSerialPort += acm_match.str();
            }
        }

        m_FeedbackFD = open(m_FeedbackSerialPort.c_str(), O_RDONLY);
        // Check if the file descriptor is pointing to a TTY device or not.
        if (!isatty(m_FeedbackFD)) {
            RCLCPP_INFO(this->get_logger(), "Feedback is not TTY device!");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Feedback is Connected on %s", m_FeedbackSerialPort.c_str());
        }

        // Get the current configuration of the serial interface
        if (tcgetattr(m_FeedbackFD, &m_FeedbackConfig) < 0) {
            RCLCPP_INFO(this->get_logger(), "Cannot get Feedback Config File!");
        }

        tcgetattr(m_FeedbackFD, &m_FeedbackConfig);

        cfsetispeed(&m_FeedbackConfig, B115200);

        tcsetattr(m_FeedbackFD, TCSANOW, &m_FeedbackConfig);

        char c;
        do {
            read(m_FeedbackFD, &c, sizeof(c));
        } while (c != '\n');

        m_FeedbackMessage.header.frame_id = "world";
        m_FeedbackMessage.name.push_back("S0");
        m_FeedbackMessage.name.push_back("S1");
        m_FeedbackMessage.name.push_back("S2");
        m_FeedbackMessage.name.push_back("S3");

        m_FeedbackMessage.position.resize(4, 0.0f);
        m_FeedbackMessage.velocity.resize(4, 0.0f);
        m_FeedbackMessage.effort.resize(4, 0.0f);

        stroke_feedback_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Feedback", 10);
        timer = this->create_wall_timer(1ms, std::bind(&SerialRX::FeedbackCallback, this));
    }

  private:
    char feedback_buffer[100];
    float feedback[4];
    void FeedbackCallback()
    {
        if (!rclcpp::ok()) {
            close(m_FeedbackFD);
            return;
        }
        memset((void*)feedback_buffer, '_' - 0, sizeof(feedback_buffer));

        uint8_t idx = 0;

		do {
            read(m_FeedbackFD, &feedback_buffer[idx], 1);
            idx++;
        } while (feedback_buffer[idx - 1] != '\n');

        uint32_t space_count = 0;
        for (char c : feedback_buffer) space_count += (c == ' ');
        if (space_count == 4) {
            sscanf(feedback_buffer, "%f %f %f %f ", &feedback[0], &feedback[1], &feedback[2], &feedback[3]);

            m_FeedbackMessage.position[0] = feedback[0];
            m_FeedbackMessage.position[1] = feedback[1];
            m_FeedbackMessage.position[2] = feedback[2];
            m_FeedbackMessage.position[3] = feedback[3];
            m_FeedbackMessage.header.stamp = this->get_clock()->now();
            stroke_feedback_publisher->publish(m_FeedbackMessage);
        }
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr stroke_feedback_publisher;
    sensor_msgs::msg::JointState m_FeedbackMessage;

    struct termios m_FeedbackConfig;

    int m_FeedbackFD;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialRX>());
    rclcpp::shutdown();
    return 0;
}
