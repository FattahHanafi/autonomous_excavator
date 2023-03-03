#include <autonomous_excavator_interfaces/action/reconstruct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

class Timer {
 public:
  Timer() { m_start = std::chrono::steady_clock::now(); }

  void reset() { m_start = std::chrono::steady_clock::now(); }

  float time() {
    m_end = std::chrono::steady_clock::now();
    return float(std::chrono::duration_cast<std::chrono::microseconds>(m_end - m_start).count());
  }

 private:
  std::chrono::steady_clock::time_point m_start;
  std::chrono::steady_clock::time_point m_end;
};

Timer timer;

namespace autonomous_excavator {
using Reconstruct = autonomous_excavator_interfaces::action::Reconstruct;
using GoalHandleReconstruct = rclcpp_action::ServerGoalHandle<Reconstruct>;
class ReconstructActionServer : public rclcpp::Node {
 public:
  explicit ReconstructActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node("camera_action_server", options) {
    using namespace std::placeholders;
    this->m_camera_server = rclcpp_action::create_server<Reconstruct>(
        this, "excavator_camera", std::bind(&ReconstructActionServer::handle_goal, this, _1, _2),
        std::bind(&ReconstructActionServer::handle_cancel, this, _1), std::bind(&ReconstructActionServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<Reconstruct>::SharedPtr m_camera_server;
  std_msgs::msg::String m_process;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const Reconstruct::Goal> goal) {
    auto feedback = std::make_shared<Reconstruct::Feedback>();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    auto feedback = std::make_shared<Reconstruct::Feedback>();
    feedback->process.data = std::string("Received cancel request");
    goal_handle->publish_feedback(feedback);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&ReconstructActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleReconstruct> goal_handle) {
    auto feedback = std::make_shared<Reconstruct::Feedback>();

    // Curve Approximation
    if (goal_handle->is_canceling()) return;
    feedback->process.data = "Started Curve Approximation";
    goal_handle->publish_feedback(feedback);
    // do Curve Approximation
    feedback->process.data = "Finished Curve Approximation";
    goal_handle->publish_feedback(feedback);

    // Surface Reconstruction
    if (goal_handle->is_canceling()) return;
    feedback->process.data = "Started Curve Approximation";
    goal_handle->publish_feedback(feedback);
    // do Surface Reconstruction
    feedback->process.data = "Finished Curve Approximation";
    goal_handle->publish_feedback(feedback);

    // Marching Cube Generation
    if (goal_handle->is_canceling()) return;
    feedback->process.data = "Started Curve Approximation";
    goal_handle->publish_feedback(feedback);
    // do Marching Cubes
    feedback->process.data = "Finished Curve Approximation";
    goal_handle->publish_feedback(feedback);

    // Volume Estimation
    if (goal_handle->is_canceling()) return;
    feedback->process.data = "Started Curve Approximation";
    goal_handle->publish_feedback(feedback);
    // do Volume Estimation
    feedback->process.data = "Finished Curve Approximation";
    goal_handle->publish_feedback(feedback);

    // Publishing Results
    if (goal_handle->is_canceling()) return;
    feedback->process.data = "Started Curve Approximation";
    goal_handle->publish_feedback(feedback);
    // do Result Piblishing
    feedback->process.data = "Finished Curve Approximation";
    goal_handle->publish_feedback(feedback);

    auto result = std::make_shared<Reconstruct::Result>();
    result->volume = 150.0;
    goal_handle->succeed(result);
  }
};
}  // namespace autonomous_excavator

RCLCPP_COMPONENTS_REGISTER_NODE(autonomous_excavator::ReconstructActionServer)
