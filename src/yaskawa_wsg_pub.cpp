#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>

using namespace std::chrono_literals;
class YaskawaWsgPub : public rclcpp::Node
{
public:
    YaskawaWsgPub() : Node("yaskawa_wsg_pub")
    {
        // Create publisher
        full_joints_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        timer_ = this->create_wall_timer(25ms, std::bind(&YaskawaWsgPub::timer_callback, this));

        this->gripper_joint = std::make_shared<sensor_msgs::msg::JointState>();
        this->full_joints = std::make_shared<sensor_msgs::msg::JointState>();

        // Create subscribers
        motoman_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motoman/joint_states", 10, std::bind(&YaskawaWsgPub::motomanJointCallback, this, std::placeholders::_1));
        gripper_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/wsg/joint_states", 10, std::bind(&YaskawaWsgPub::gripperJointCallback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        if(!this->joints_read || !this->gripper_read)
        {
            return;
        }
        full_joints->name.back() = this->gripper_joint->name.at(0);
        full_joints->position.back() = this->gripper_joint->position.at(0);
        full_joints->header.stamp = this->get_clock()->now(); // ros::Time::now();
        // std::cout << "Complete message:" << std::endl;
        // RCLCPP_INFO_STREAM(this->get_logger(), sensor_msgs::msg::to_yaml(*full_joints));
        full_joints_pub->publish(*this->full_joints);
    }

    void motomanJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        this->joints_read = true;
        full_joints->name.resize(msg->name.size() + 1);
        full_joints->position.resize(msg->position.size() + 1);

        for (size_t i = 0; i < msg->position.size(); i++)
        {
            full_joints->name.at(i) = msg->name.at(i);
            full_joints->position.at(i) = msg->position.at(i);
        }
    }

    void gripperJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        this->gripper_read = true;
        double gripper_joint_value_ = msg->position.at(0) / 2 - 0.034;

        this->gripper_joint->name.resize(1);
        this->gripper_joint->position.resize(1);

        this->gripper_joint->name.at(0) = msg->name.at(0);
        this->gripper_joint->position.at(0) = gripper_joint_value_;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr full_joints_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motoman_joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_sub_;
    sensor_msgs::msg::JointState::SharedPtr gripper_joint;
    sensor_msgs::msg::JointState::SharedPtr full_joints;
    bool gripper_read = false;
    bool joints_read = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YaskawaWsgPub>());
    rclcpp::shutdown();
    return 0;
}
