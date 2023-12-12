#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <eigen3/Eigen/Geometry>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "uclv_utilities/utilities.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This class is a control node for the force reference by applying velocities command.
    Input - measured force
    Output - velocities command
*/

class ForceControlNode : public rclcpp::Node
{
public:
    ForceControlNode()
        : Node("force_control_node")
    {
        measure_topic_name_ = this->declare_parameter<std::string>("measure_topic_name_", "force_measured");
        reference_topic_name_ = this->declare_parameter<std::string>("reference_topic_name_", "force_reference");
        control_topic_name_ = this->declare_parameter<std::string>("control_topic_name_", "force_control_twist");
        robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame_", "base_link");
        sensor_frame_ = this->declare_parameter<std::string>("sensor_frame_", "sensor_frame");
        contact_frame_ = this->declare_parameter<std::string>("contact_frame_", "contact_frame");
        robot_pusher_frame_ = this->declare_parameter<std::string>("robot_pusher_frame_", "push_extension");
        control_gain_ = this->declare_parameter<double>("control_gain_", 0.8);
        loop_time = this->declare_parameter<double>("loop_time", 0.01);

        // Initiliaze variables
        error = 0;
        measure = 0;
        reference = 0;

        this->time_actual = this->now().seconds();
        this->init = true;

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(control_topic_name_, 1);

        force_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            measure_topic_name_, 1, std::bind(&ForceControlNode::loop_measure_cb, this, _1));

        force_ref_sub = this->create_subscription<std_msgs::msg::Float64>(
            reference_topic_name_, 1, std::bind(&ForceControlNode::loop_reference_cb, this, _1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(loop_time), std::bind(&ForceControlNode::timer_callback, this));
    }

private:
    void loop_measure_cb(const geometry_msgs::msg::WrenchStamped &measure)
    {
        // Get the measure in sensor frame
        // std::cout << BOLDCYAN << "Received measure \n" << RESET << std::endl;
        RCLCPP_INFO_STREAM(this->get_logger(), measure.wrench.force.x << " " << measure.wrench.force.y << " " << measure.wrench.force.z);

        this->measure_timestamp = rclcpp::Time(measure.header.stamp.sec, measure.header.stamp.nanosec);

        S_f << measure.wrench.force.x, measure.wrench.force.y, measure.wrench.force.z;

        // Get the transform from the contact_frame to the sensor_frame
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(
                contact_frame_, sensor_frame_,
                measure_timestamp, 50ms);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                sensor_frame_.data(), contact_frame_.data(), ex.what());
            return;
        }

        Eigen::Isometry3d C_T_S = uclv::geometry_2_eigen(t.transform);

        // Get the measure in contact frame
        C_f = C_T_S.rotation() * S_f;

        this->measure = C_f(0);
    }

    void loop_reference_cb(const std_msgs::msg::Float64 &reference_msg)
    {
        // std::cout << BOLDCYAN << "Received reference \n" << RESET << std::endl;
        RCLCPP_INFO_STREAM(this->get_logger(), reference_msg.data);
        this->reference = reference_msg.data;
    }

    void timer_callback()
    {
        geometry_msgs::msg::TwistStamped vel_ref_msg;

        error = reference - measure;

        // Evaluate velocity in the contact frame
        C_v << control_gain_ * error, 0, 0;

        // Convert velocity from contact frame into base_link frame
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(
                robot_base_frame_, contact_frame_,
                measure_timestamp, 50ms);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                robot_base_frame_.data(), contact_frame_.data(), ex.what());
            return;
        }

        W_T_C = uclv::geometry_2_eigen(t.transform);
        W_v = W_T_C.rotation() * C_v;

        vel_ref_msg.header.stamp = this->now();
        vel_ref_msg.header.frame_id = robot_base_frame_;
        vel_ref_msg.twist.linear.x = W_v(0);
        vel_ref_msg.twist.linear.y = W_v(1);
        vel_ref_msg.twist.linear.z = W_v(2);
        vel_ref_msg.twist.angular.x = 0;
        vel_ref_msg.twist.angular.y = 0;
        vel_ref_msg.twist.angular.z = 0;

        vel_pub->publish(vel_ref_msg);
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr force_ref_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    // Tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Parameters
    std::string measure_topic_name_;
    std::string reference_topic_name_;
    std::string control_topic_name_;
    std::string robot_base_frame_;   // Frame where the Jacobian is computed
    std::string sensor_frame_;       // Frame until the Jacobian is computed
    std::string robot_pusher_frame_; // Frame where the velocity is applied
    std::string contact_frame_;
    Eigen::Isometry3d W_T_C; // Transform from contact frame to base_link frame

    // Controller parameters
    double control_gain_ = 0;
    double loop_time = 0.001;

    // Variables
    double measure;   // Measured force
    double reference; // Reference force
    Eigen::Vector3d S_f;              // Measured force in sensor frame
    Eigen::Vector3d C_f;              // Measured force in contact frame
    Eigen::Vector3d C_v;              // Velocity in contact frame
    Eigen::Vector3d W_v;              // Velocity in base_link frame
    rclcpp::Time measure_timestamp;
    double error = 0;
    double time_actual = 0;
    bool init = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceControlNode>());
    rclcpp::shutdown();
    return 0;
}