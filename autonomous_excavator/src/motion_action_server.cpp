#include <autonomous_excavator_interfaces/action/motion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

namespace autonomous_excavator {
using Camera = autonomous_excavator_interfaces::action::Motion;
using GoalHandleCamera = rclcpp_action::ServerGoalHandle<Camera>;
class CameraActionServer : public rclcpp::Node {
 public:
  explicit CameraActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("motion_action_server", options) {
    using namespace std::placeholders;
    this->m_camera_server = rclcpp_action::create_server<Camera>(this, "excavator_motion", std::bind(&CameraActionServer::handle_goal, this, _1, _2),
                                                                 std::bind(&CameraActionServer::handle_cancel, this, _1),
                                                                 std::bind(&CameraActionServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<Camera>::SharedPtr m_camera_server;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Camera::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCamera> goal_handle) { return rclcpp_action::CancelResponse::ACCEPT; }

  void handle_accepted(const std::shared_ptr<GoalHandleCamera> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&CameraActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCamera> goal_handle) {
    auto feedback = std::make_shared<Camera::Feedback>();
    goal_handle->publish_feedback(feedback);
    auto result = std::make_shared<Camera::Result>();
    goal_handle->succeed(result);
  }
};
}  // namespace autonomous_excavator

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_excavator::CameraActionServer)
