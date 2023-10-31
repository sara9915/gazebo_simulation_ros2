#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class YaskawaWsgPub : public rclcpp::Node
{
public:
    YaskawaWsgPub() : Node("yaskawa_wsg_pub")
    {
        // Create publisher
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

        // Create subscribers
        motoman_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/motoman/joint_states", 10, std::bind(&YaskawaWsgPub::motomanJointCallback, this, std::placeholders::_1));
        gripper_joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&YaskawaWsgPub::gripperJointCallback, this, std::placeholders::_1));
    }

private:
    void motomanJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Extract joint values from received message
        // std::vector<double> motoman_joint_values;
        // for (size_t i = 0; i < msg->name.size(); ++i)
        // {
        //     if (msg->name[i].find("gripper_joint") == std::string::npos)
        //     {
        //         motoman_joint_values.push_back(msg->position[i]);
        //     }
        // }

        // // Update joint state message
        // joint_state_msg_.position = motoman_joint_values;
        // joint_state_msg_.header.stamp = this->now();

        // Publish joint state message
        joint_state_pub_->publish(*msg);
    }

    void gripperJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        double gripper_joint_value = msg->position[0]/2 - 0.034;
        // Extract gripper joint value from received message
        // double gripper_joint_value = 0.0;
        // for (size_t i = 0; i < msg->name.size(); ++i)
        // {
        //     if (msg->name[i] == "gripper_joint")
        //     {
        //         gripper_joint_value = msg->position[i];
        //         break;
        //     }
        // }

        // // Update joint state message
        joint_state_msg_.position.push_back(gripper_joint_value);
        joint_state_msg_.header.stamp = this->now();

        // Publish joint state message
        joint_state_pub_->publish(joint_state_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr motoman_joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_joint_sub_;
    sensor_msgs::msg::JointState joint_state_msg_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YaskawaWsgPub>());
    rclcpp::shutdown();
    return 0;
}
