#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "MarchingCubes.hpp"

class MarchingCubeNode : public rclcpp::Node {
 public:
  MarchingCubeNode() : Node("marching_cube_node") {
    MarchingCubes m_marchingCubes(100, 100, 100, 0.1);
    m_marchingCubes.setSize(vec3f{0.2, 0.2, 0.2});
    RCLCPP_INFO(this->get_logger(), "%s", m_marchingCubes.print().c_str());
  }

 private:
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MarchingCubeNode>());
  rclcpp::shutdown();
  return 0;
}
