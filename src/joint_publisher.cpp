#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define M_PI 3.14159265358979323846
#define m 2
#define theta1 52.13 * M_PI / 180.0
#define theta3 45.52 * M_PI / 180.0
#define L1 199.53 * m
#define L2 258.36 * m
#define L3 100 * m
#define L4 220 * m
#define L5 223.595 * m
#define L6 122.5 * m
#define L7 140 * m
#define theta6 30.58 * M_PI / 180.0
#define theta4 11.31 * M_PI / 180.0
#define L11 280 * m
#define L12 147 * m
#define L9 110 * m
#define L13 70 * m
#define L14 192.5 * m
#define L15 70 * m
#define theta19 106.87 * M_PI / 180.0
#define LL1 535.395 * m
#define LL2 280 * m
#define LL3 174.5 * m

using namespace std::chrono_literals;
using std::placeholders::_1;

class JointStatePiblisher : public rclcpp::Node {
  public:
    JointStatePiblisher() : Node("joint_state_publisher")
    {
        stroke_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "Machine/ActuatorStroke/Feedback", 10, std::bind(&JointStatePiblisher::StrokeFeedbackCallback, this, _1));
        joint_angle_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_angle_message.name.resize(4);
        joint_angle_message.position.resize(4, 0.0f);
        joint_angle_message.velocity.resize(4, 0.0f);
        joint_angle_message.effort.resize(4, 0.0f);

        joint_angle_message.name[0] = "Swing";
        joint_angle_message.name[1] = "Boom";
        joint_angle_message.name[2] = "Arm";
        joint_angle_message.name[3] = "Bucket";
    }

  private:
    void StrokeFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        const float S1 = msg->position[0];
        const float S2 = msg->position[1];
        const float S3 = msg->position[2];

        float theta2 = acos((L1 * L1 + L2 * L2 - S1 * S1) / (2 * L1 * L2));
        float q1 = M_PI - theta1 - theta2 - theta3;
        float theta5 = acos((L4 * L4 + L3 * L3 - S2 * S2) / (2 * L3 * L4)) - theta4;
        float q2 = M_PI - theta5;
        float theta7 = acos((L6 * L6 + L5 * L5 - S3 * S3) / (2 * L6 * L5));
        float theta8 = M_PI - theta7 - theta6;
        float L8 = sqrt(L6 * L6 + L7 * L7 - 2 * L6 * L7 * cos(theta8));
        float L10 = sqrt((L3 + L9) * (L3 + L9) + L7 * L7 - 2 * (L3 + L9) * L7 * cos(theta5 + theta4));
        float theta9 = acos((L7 * L7 + L10 * L10 - (L3 + L9) * (L3 + L9)) / (2 * L7 * L10));
        float theta10 = M_PI - theta9 - theta5 - theta4;
        float a = sqrt(L6 * L6 + L10 * L10 - 2 * L6 * L10 * cos(theta8 + theta9));
        float theta11 = acos((L10 * L10 + a * a - L6 * L6) / (2 * L10 * a));
        float theta12 = acos((L12 * L12 + a * a - L11 * L11) / (2 * L12 * a));
        float theta13 = theta11 + theta12 - theta10;
        float theta14 = M_PI - theta13;
        float b = sqrt(L12 * L12 + L13 * L13 - 2 * L12 * L13 * cos(theta14));
        float theta15 = acos((L13 * L13 + b * b - L12 * L12) / (2 * L13 * b));
        float theta16 = acos((L15 * L15 + b * b - L14 * L14) / (2 * L15 * b));
        float theta17 = theta15 + theta16;
        float theta18 = M_PI - theta17;
        float q3 = theta19 - theta18;
        float c = sqrt(LL2 * LL2 + LL3 * LL3 - 2 * LL2 * LL3 * cos(M_PI - q3));
        float theta20 = acos((LL2 * LL2 + c * c - LL3 * LL3) / (2 * LL2 * c));
        float theta21 = M_PI - theta20 - q2;
        float REP = sqrt(LL1 * LL1 + c * c - 2 * LL1 * c * cos(theta21));
        float theta22 = acos((REP * REP + LL1 * LL1 - c * c) / (2 * LL1 * REP));
        float theta23 = q1 - theta22;
        float EPX = REP * cos(theta23);
        float EPZ = REP * sin(theta23);
        float Theta_end = q1 - q2 - q3;

        joint_angle_message.position[0] = 0.0;
        joint_angle_message.position[1] = -q1;// * -q1;
        joint_angle_message.position[2] = q2;// q2;  // * 0.3 + 0.2 * std::sin(2 * now.seconds());
        joint_angle_message.position[3] = q3;//  + theta19 - M_PI / 2.0 ;// q3;     // * 0.5 + 0.2 * std::sin(2 * now.seconds());
        joint_angle_message.header.stamp = this->get_clock()->now();
        joint_angle_publisher->publish(joint_angle_message);
    }

    sensor_msgs::msg::JointState joint_angle_message;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr stroke_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_angle_publisher;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatePiblisher>());
    rclcpp::shutdown();
    return 0;
}
