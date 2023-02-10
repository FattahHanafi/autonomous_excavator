#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class UdpInterface : public rclcpp::Node {
 public:
  explicit UdpInterface() : Node("udp_interface") {
    if ((m_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP) < 0)) {
      RCLCPP_WARN(this->get_logger(), "Socke creation failed\n");
      return;
    }
    memset(&m_server_address, 0, sizeof(m_server_address));
    m_server_address.sin_family = AF_INET;
    m_server_address.sin_port = htons(5000);
    //m_server_address.sin_addr.s_addr = inet_addr("192.168.1.9");
    m_server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    // inet_addr("10.2.134.10");
    // m_server_address.sin_addr.s_addr = INADDR_ANY;
    // m_server_address.sin_addr.s_addr = 0xc0a80109;
    // inet_aton("192.168.1.9", &m_server_address.sin_addr);
    bind(m_sockfd, &m_server_addr)

    FeedbackCallback();
  }

 private:
  void FeedbackCallback() {
    int n;
    socklen_t len;
    while (rclcpp::ok()) {
      n = recvfrom(m_sockfd, (char *)m_rx_buffer, 80, MSG_WAITALL, (struct sockaddr *)&m_server_address, &len);
      m_rx_buffer[n] = '\n';
      RCLCPP_INFO(this->get_logger(), "Recieved %i bytes", n);
    }
  }

  rclcpp::TimerBase::SharedPtr timer;
  std_msgs::msg::Float32MultiArray m_rx_message;
  int m_sockfd;
  char m_rx_buffer[1000];
  struct sockaddr_in m_server_address;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UdpInterface>());
  rclcpp::shutdown();
  return 0;
}
