#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include <eigen3/Eigen/Geometry>
#include "uclv_utilities/utilities.hpp"

using std::placeholders::_1;

/* This class is a control node for the angle tracking.
    Input - measured angle
    Output - Twist end effector tool0
*/

class AngleControlNode : public rclcpp::Node
{
public:
    AngleControlNode()
        : Node("angle_control_node")
    {
        measure_topic_name_ = this->declare_parameter<std::string>("measure_topic_name_", "angle_measure");
<<<<<<< HEAD
        control_topic_name_ = this->declare_parameter<std::string>("control_topic_name_", "twist_controller");
        robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame_", "base_link");
        robot_ee_frame_ = this->declare_parameter<std::string>("robot_ee_frame_", "tool0");
        robot_pusher_frame_ = this->declare_parameter<std::string>("robot_pusher_frame_", "push_extension");
        control_gain_ = this->declare_parameter<double>("control_gain_", 0.1);
=======
        control_topic_name_ = this->declare_parameter<std::string>("control_topic_name_", "angle_control_twist");
        robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame_", "base_link");
        robot_ee_frame_ = this->declare_parameter<std::string>("robot_ee_frame_", "tool0");
        robot_pusher_frame_ = this->declare_parameter<std::string>("robot_pusher_frame_", "push_extension");
        control_gain_ = this->declare_parameter<double>("control_gain_", 0.8);

        this->time_actual = this->now().seconds();
        this->init = true;
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c

        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>(control_topic_name_, 1);

        angle_sub = this->create_subscription<std_msgs::msg::Float64>(
            measure_topic_name_, 1, std::bind(&AngleControlNode::loop_cb, this, _1));
    }

private:
    void loop_cb(const std_msgs::msg::Float64 &angle) 
    {
<<<<<<< HEAD
        RCLCPP_INFO(this->get_logger(), "Received Measure: '%f'", angle.data);
=======
        // RCLCPP_INFO(this->get_logger(), "Received Measure: '%f'", angle.data);
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c

        theta_dot = -control_gain_ * angle.data;

        // Get the transform from the base_link to the pusher frame
        geometry_msgs::msg::TransformStamped t;
        try
        {
            t = tf_buffer_->lookupTransform(
                robot_base_frame_, robot_pusher_frame_, 
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                robot_pusher_frame_.data(), robot_base_frame_.data(), ex.what());
            return;
        }

        W_T_P = uclv::geometry_2_eigen(t.transform);

<<<<<<< HEAD
        P_w_p << 0, 0, theta_dot;
=======
        // Evaluate velocity of the pusher in the world frame
        double dt = (this->now().seconds() - this->time_actual);
        if(this->init)
        {
            W_pold_P = W_T_P.translation();
            this->init = false;
        }
        W_pdot_P = (W_T_P.translation()-W_pold_P)/0.1;
        W_pold_P = W_T_P.translation();

        // print W_pdot_P
        // std::cout << BOLDRED << "W_p_P: \n" << RESET << W_T_P.translation() << std::endl;
        // std::cout << BOLDRED << "W_pdot_P (dt = " << BOLDWHITE << dt << BOLDRED "): \n" << RESET << W_pdot_P <<  std::endl;

        
        // save actual time
        this->time_actual = this->now().seconds();


        P_w_p << 0, 0, -theta_dot;
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
        W_w_p =  W_T_P.rotation()* P_w_p;

        // get the transform from pusher frame to ee frame and get the translation, then evaluate from the translation the skew matrix and multiply it by the vector v
        try
        {
            t = tf_buffer_->lookupTransform(
                robot_pusher_frame_, robot_ee_frame_,
                tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                robot_ee_frame_.data(), robot_pusher_frame_.data(), ex.what());
            return;
        }

        P_T_T0 = uclv::geometry_2_eigen(t.transform);
    
        P_o_T0 << P_T_T0.translation().x(), P_T_T0.translation().y(), P_T_T0.translation().z();

        skew_mat << 0, -P_o_T0(2), P_o_T0(1),
<<<<<<< HEAD
            P_o_T0(2), 0, -P_o_T0(0),
            -P_o_T0(1), P_o_T0(0), 0;
        
        W_pdot_T0 = -skew_mat * W_w_p;
=======
                    P_o_T0(2), 0, -P_o_T0(0),
                    -P_o_T0(1), P_o_T0(0), 0;
        
        W_pdot_T0 = -W_pdot_P+skew_mat * W_w_p;
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
        W_w_T0 = W_w_p;

        // Publish the twist
        twist.header.stamp = this->now();
        twist.header.frame_id = robot_base_frame_;
        twist.twist.linear.x = W_pdot_T0(0);
        twist.twist.linear.y = W_pdot_T0(1);
        twist.twist.linear.z = W_pdot_T0(2);
        twist.twist.angular.x = W_w_T0(0);
        twist.twist.angular.y = W_w_T0(1);
        twist.twist.angular.z = W_w_T0(2);

        twist_pub->publish(twist);
    }

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub;

    // Tf2
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Parameters
    std::string measure_topic_name_;
    std::string control_topic_name_;
    std::string robot_base_frame_;   // Frame where the Jacobian is computed
    std::string robot_ee_frame_;     // Frame until the Jacobian is computed
    std::string robot_pusher_frame_; // Frame where the velocity is applied

    // Controller parameters
    double control_gain_ = 0;

    // Variables
    geometry_msgs::msg::TwistStamped twist; // Controller output
    Eigen::Matrix3d skew_mat;
<<<<<<< HEAD
=======
    Eigen::Vector3d W_pdot_P;
    Eigen::Vector3d W_pold_P;
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
    Eigen::Vector3d W_pdot_T0;
    Eigen::Vector3d W_w_T0;
    Eigen::Vector3d P_w_p;
    Eigen::Vector3d W_w_p;
    Eigen::Isometry3d W_T_P;
    Eigen::Isometry3d P_T_T0;
    Eigen::Vector3d P_o_T0;
    double theta_dot=0;
<<<<<<< HEAD
=======
    double time_actual=0;
    bool init = false;
>>>>>>> 14fa28f5b0662a0ff8006b37cbcfd76033c1e09c
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleControlNode>());
    rclcpp::shutdown();
    return 0;
}