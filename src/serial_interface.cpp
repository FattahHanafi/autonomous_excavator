#include <errno.h>  // Error integer and strerror() function
#include <fcntl.h>  // Contains file controls like O_RDWR
#include <stdio.h>
#include <string.h>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <algorithm>
#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <stdexcept>
#include <string>

#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class SerialInterface : public rclcpp::Node {
  public:
    explicit SerialInterface() : Node("serial_interface")
    {
        std::string m_CommandSerialPort = "/dev/ttyACM";
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
                if (ser_match.str() == "85935333637351C0E121") m_CommandSerialPort += acm_match.str();
                if (ser_match.str() == "75130303236351612250") m_FeedbackSerialPort += acm_match.str();
            }
        }

        // m_CommandFD = open(m_CommandSerialPort, O_WRONLY | O_NOCTTY | O_NDELAY);
        // m_FeedbackFD = open(m_FeedbackSerialPort, O_RDONLY | O_NOCTTY | O_NDELAY);

        m_CommandFD = open(m_CommandSerialPort.c_str(), 0);
        m_FeedbackFD = open(m_FeedbackSerialPort.c_str(), 0);
        // Check if the file descriptor is pointing to a TTY device or not.
        if (!isatty(m_CommandFD)) {
            RCLCPP_INFO(this->get_logger(), "Command is not TTY device!");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Command is Connected on %s", m_CommandSerialPort.c_str());
        }
        if (!isatty(m_FeedbackFD)) {
            RCLCPP_INFO(this->get_logger(), "Feedback is not TTY device!");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Feedback is Connected on %s", m_FeedbackSerialPort.c_str());
        }

        // Get the current configuration of the serial interface
        if (tcgetattr(m_CommandFD, &m_CommandConfig) < 0) {
            RCLCPP_INFO(this->get_logger(), "Cannot get Command Config File!");
        }
        if (tcgetattr(m_FeedbackFD, &m_FeedbackConfig) < 0) {
            RCLCPP_INFO(this->get_logger(), "Cannot get Feedback Config File!");
        }

        tcgetattr(m_CommandFD, &m_CommandConfig);
        tcgetattr(m_FeedbackFD, &m_FeedbackConfig);

        // m_FeedbackConfig.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
        // m_CommandConfig.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);

        cfsetospeed(&m_CommandConfig, B115200);
        cfsetispeed(&m_FeedbackConfig, B115200);

        tcsetattr(m_CommandFD, TCSAFLUSH, &m_CommandConfig);
        tcsetattr(m_FeedbackFD, TCSAFLUSH, &m_FeedbackConfig);

        m_FeedbackMessage.header.frame_id = "world";
        m_FeedbackMessage.name.push_back("S1");
        m_FeedbackMessage.name.push_back("S2");
        m_FeedbackMessage.name.push_back("S3");

        m_FeedbackMessage.position.resize(3, 0.0f);
        m_FeedbackMessage.velocity.resize(3, 0.0f);
        m_FeedbackMessage.effort.resize(3, 0.0f);

        stroke_feedback_publisher = this->create_publisher<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Feedback", 10);
        stroke_command_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Command", 10,
                                                                                            std::bind(&SerialInterface::CommandCallback, this, _1));
        timer = this->create_wall_timer(10ms, std::bind(&SerialInterface::FeedbackCallback, this));
    }

  private:
    void CommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        uint32_t S1 = uint32_t(msg->position[0]);
        uint32_t S2 = uint32_t(msg->position[1]);
        uint32_t S3 = uint32_t(msg->position[2]);
        /* S123=S2*1000000+S3*1000+S1;
        process='digging';
        data="<"+process+","+S123+","+S4+">"; */

        std::string stroke_command = "<dumping," + std::to_string(S2) + std::to_string(S3) + std::to_string(S1) + ",0>\n";
        // RCLCPP_INFO(this->get_logger(), "Command = %s", stroke_command.c_str());
        // int written_size = write(m_CommandFD, stroke_command.c_str(), sizeof(stroke_command));
        // RCLCPP_INFO(this->get_logger(), "Written Size = %i", written_size);
    }

    char feedback_buffer[100];
    float feedback[7];
    int a;
    void FeedbackCallback()
    {
        memset((void*)feedback_buffer, 0x20, sizeof(feedback_buffer));
        int n = read(m_FeedbackFD, feedback_buffer, sizeof(feedback_buffer));
        if (n > 10) {
            uint32_t comma_count = 0;
            for (char c : feedback_buffer) comma_count += (c == ',');
            if ((comma_count == 7) || (feedback_buffer[0] != ',')) {
                sscanf(feedback_buffer, "%i,%f,%f,%f,%f,%f,%f,%f", &a, &feedback[0], &feedback[1], &feedback[2], &feedback[3], &feedback[4],
                       &feedback[5], &feedback[6]);
                m_FeedbackMessage.position[0] = feedback[0];
                m_FeedbackMessage.position[1] = feedback[1];
                m_FeedbackMessage.position[2] = feedback[2];
                m_FeedbackMessage.header.stamp = this->get_clock()->now();
                stroke_feedback_publisher->publish(m_FeedbackMessage);
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stroke_command_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr stroke_feedback_publisher;
    sensor_msgs::msg::JointState m_FeedbackMessage;

    struct termios m_CommandConfig;
    struct termios m_FeedbackConfig;

    int m_CommandFD;
    int m_FeedbackFD;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialInterface>());
    rclcpp::shutdown();
    return 0;
}
