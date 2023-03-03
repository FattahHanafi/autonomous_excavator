#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#include <memory>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node {
 public:
  explicit FramePublisher() : Node("excavator_tf2_broadcaster") {
    joint_subscription =
        this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&FramePublisher::make_transforms, this, _1));
    tf_publisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

 private:
  void make_transforms(sensor_msgs::msg::JointState::SharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();

    m_TransformStamped.header.stamp = now;

    m_TransformStamped.header.frame_id = "world";
    m_TransformStamped.child_frame_id = "container";

    m_TransformStamped.transform.translation.x = 0.0;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.0;
    m_Quaternion.setRPY(0, 0, 0);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "world";
    m_TransformStamped.child_frame_id = "base";

    m_TransformStamped.transform.translation.x = 0.0;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.0;
    m_Quaternion.setRPY(0, 0, 0);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "base";
    m_TransformStamped.child_frame_id = "swing";

    m_TransformStamped.transform.translation.x = 0.2548;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.247;
    m_Quaternion.setRotation(tf2::Vector3(0, 0, 1), msg->position[0]);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "swing";
    m_TransformStamped.child_frame_id = "boom";
    m_TransformStamped.transform.translation.x = 0.3445;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.063;
    m_Quaternion.setRotation(tf2::Vector3(0, 1, 0), msg->position[1]);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "boom";
    m_TransformStamped.child_frame_id = "arm";
    m_TransformStamped.transform.translation.x = 1.071;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.0;
    m_Quaternion.setRotation(tf2::Vector3(0, 1, 0), msg->position[2]);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "arm";
    m_TransformStamped.child_frame_id = "bucket";
    m_TransformStamped.transform.translation.x = 0.560;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.0;
    m_Quaternion.setRotation(tf2::Vector3(0, 1, 0), msg->position[3]);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);

    m_TransformStamped.header.frame_id = "bucket";
    m_TransformStamped.child_frame_id = "teeth";
    m_TransformStamped.transform.translation.x = 0.0;
    m_TransformStamped.transform.translation.y = 0.0;
    m_TransformStamped.transform.translation.z = 0.0;
    m_Quaternion.setRotation(tf2::Vector3(0, 0, 1), 0);
    m_TransformStamped.transform.rotation.x = m_Quaternion.x();
    m_TransformStamped.transform.rotation.y = m_Quaternion.y();
    m_TransformStamped.transform.rotation.z = m_Quaternion.z();
    m_TransformStamped.transform.rotation.w = m_Quaternion.w();

    tf_publisher->sendTransform(m_TransformStamped);
  }

  geometry_msgs::msg::TransformStamped m_TransformStamped;
  tf2::Quaternion m_Quaternion;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
