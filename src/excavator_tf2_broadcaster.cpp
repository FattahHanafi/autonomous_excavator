#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <memory>

using std::placeholders::_1;

class FramePublisher : public rclcpp::Node
{
public:
    explicit FramePublisher() : Node("excavator_tf2_broadcaster")
    {
        joint_subscription = this->create_subscription<sensor_msgs::msg::JointState>("Machine/JointState", 10, std::bind(&FramePublisher::make_transforms, this, _1));
        tf_publisher = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

private:
    void make_transforms(sensor_msgs::msg::JointState::SharedPtr msg)
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;
        tf2::Quaternion q;
        tf2::Quaternion joint;

        t.header.stamp = now;
        t.header.frame_id = "world";
        t.child_frame_id = "base";

        t.transform.translation.x = 2.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 1.0;
        q.setRPY(0, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_publisher->sendTransform(t);

        t.header.frame_id = "base";
        t.child_frame_id = "swing";

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        joint.setRotation(tf2::Vector3(0, 0, 1), msg->position[0]);
        t.transform.rotation.x = joint.x();
        t.transform.rotation.y = joint.y();
        t.transform.rotation.z = joint.z();
        t.transform.rotation.w = joint.w();

        tf_publisher->sendTransform(t);

        t.header.frame_id = "swing";
        t.child_frame_id = "boom";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        joint.setRotation(tf2::Vector3(0, 1, 0), msg->position[1]);
        t.transform.rotation.x = joint.x();
        t.transform.rotation.y = joint.y();
        t.transform.rotation.z = joint.z();
        t.transform.rotation.w = joint.w();

        tf_publisher->sendTransform(t);

        t.header.frame_id = "boom";
        t.child_frame_id = "arm";
        t.transform.translation.x = 1.5;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        joint.setRotation(tf2::Vector3(0, 1, 0), msg->position[2]);
        t.transform.rotation.x = joint.x();
        t.transform.rotation.y = joint.y();
        t.transform.rotation.z = joint.z();
        t.transform.rotation.w = joint.w();
        
		tf_publisher->sendTransform(t);

		t.child_frame_id = "camera";
		t.transform.translation.x = 0.5;
		t.transform.translation.y = 0.0;
		t.transform.translation.z = 0.0;
		t.transform.rotation.x = 0.0;
		t.transform.rotation.y = 0.0;
		t.transform.rotation.z = 0.0;
		tf_publisher->sendTransform(t);


        t.header.frame_id = "arm";
        t.child_frame_id = "bucket";
        t.transform.translation.x = 1.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        joint.setRotation(tf2::Vector3(0, 1, 0), msg->position[3]);
        t.transform.rotation.x = joint.x();
        t.transform.rotation.y = joint.y();
        t.transform.rotation.z = joint.z();
        t.transform.rotation.w = joint.w();

        tf_publisher->sendTransform(t);

        t.header.frame_id = "bucket";
        t.child_frame_id = "teeth";
        t.transform.translation.x = 0.5;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        joint.setRotation(tf2::Vector3(0, 0, 1), 0);
        t.transform.rotation.x = joint.x();
        t.transform.rotation.y = joint.y();
        t.transform.rotation.z = joint.z();
        t.transform.rotation.w = joint.w();

        tf_publisher->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
