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

class SerialTX : public rclcpp::Node {
  public:
    explicit SerialTX() : Node("serial_tx")
    {
        std::string m_CommandSerialPort = "/dev/ttyACM";

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
                if (ser_match.str() == "75130303236351612250") m_CommandSerialPort += acm_match.str();
            }
        }

        m_CommandFD = open(m_CommandSerialPort.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);

        // Check if the file descriptor is pointing to a TTY device or not.
        if (!isatty(m_CommandFD)) {
            RCLCPP_INFO(this->get_logger(), "Command is not TTY device!");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Command is Connected on %s", m_CommandSerialPort.c_str());
        }

        // Get the current configuration of the serial interface
        if (tcgetattr(m_CommandFD, &m_CommandConfig) < 0) {
            RCLCPP_INFO(this->get_logger(), "Cannot get Command Config File!");
        }

        tcgetattr(m_CommandFD, &m_CommandConfig);
        cfmakeraw(&m_CommandConfig);
        cfsetospeed(&m_CommandConfig, B115200);
        cfsetispeed(&m_CommandConfig, B115200);
        m_CommandConfig.c_cflag = (m_CommandConfig.c_cflag & ~CSIZE) | CS8;
        m_CommandConfig.c_iflag &= ~IGNBRK;  // disable break processing
        m_CommandConfig.c_lflag = 0;         // no signaling chars, no echo, no canonical processing
        m_CommandConfig.c_oflag = 0;         // no remapping, no delays
        m_CommandConfig.c_cc[VMIN] = 0;      // read doesn't block
        m_CommandConfig.c_cc[VTIME] = 5;     // 0.5 seconds read timeout

        m_CommandConfig.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

        m_CommandConfig.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
        m_CommandConfig.c_cflag &= ~(PARENB | PARODD);  // shut off parity
        m_CommandConfig.c_cflag |= 0;
        m_CommandConfig.c_cflag &= ~CSTOPB;
        m_CommandConfig.c_cflag &= ~CRTSCTS;
        //
        m_CommandConfig.c_oflag &= ~(IXOFF | IXANY);
        //
        cfsetospeed(&m_CommandConfig, B115200);

        tcsetattr(m_CommandFD, TCSANOW, &m_CommandConfig);

        auto start = std::chrono::steady_clock::now();
        auto end = std::chrono::steady_clock::now();
        float delay = 0;
        RCLCPP_INFO(this->get_logger(), "Setting serial config...");
        while (delay < 5.0f) {
            end = std::chrono::steady_clock::now();
            delay = std::chrono::duration_cast<std::chrono::seconds>(end - start).count();
        }
        RCLCPP_INFO(this->get_logger(), "Set serial config.");

        stroke_command_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("Machine/ActuatorStroke/Command", 10,
                                                                                            std::bind(&SerialTX::CommandCallback, this, _1));
    }

  private:
    void CommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!rclcpp::ok()) {
            close(m_CommandFD);
            return;
        }
        std::string stroke_command = std::to_string(msg->position[0]) + ' ' + std::to_string(msg->position[1]) + ' ' +
                                     std::to_string(msg->position[2]) + ' ' + std::to_string(msg->position[3]) + " \n";
        write(m_CommandFD, stroke_command.c_str(), stroke_command.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stroke_command_subscriber;

    struct termios m_CommandConfig;

    int m_CommandFD;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialTX>());
    rclcpp::shutdown();
    return 0;
}
