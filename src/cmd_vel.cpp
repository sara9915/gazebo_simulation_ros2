// This node allows to command the robot via velocity commands.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "uclv_moveit_planner_ros2/planner.h"

bool activate_srv = false;

bool activate_srv_callback(std_srvs::SetBool::Request &req,
                           std_srvs::SetBool::Response &res)
{
    res.success = true;
    activate_srv = req.data;
    return true;
}


void update_vel(const geometry_msgs::Twist::ConstPtr &msg, Eigen::Matrix<double, 6, 1> *vipi)
{
    *vipi << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z; // desired pusher velocity in world frame
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_command_vel_as");
    ros::NodeHandle nh;

    // MoveIt variables (used to calculate Jacobian)
    static const std::string PLANNING_GROUP = "yaskawa_arm";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO_STREAM("Model frame: " << kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

    // Useful variables for Euler integration
    Eigen::Matrix<double, 6, 1> vipi; vipi << 0,0,0,0,0,0;
    Eigen::Matrix<double,7,1> q_dot; q_dot << 0,0,0,0,0,0,0;
    Eigen::Matrix<double,7,1> q; q << 0,0,0,0,0,0,0;
    Eigen::MatrixXd jacobian;
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::VectorXd joint_values;
    double euler_frequency = 40; // [Hz]
    double euler_sample_time = 1 / euler_frequency;
    ros::Rate loop(euler_frequency);

    // Service
    ros::ServiceServer service =
        nh.advertiseService("command_vel", activate_srv_callback);

    while (!activate_srv)
    {
        ros::spinOnce();
        loop.sleep();
        ROS_INFO_STREAM(activate_srv);
        // Wait until the service being active
    }

    // Reading robot configuration q0
    auto q0 = ros::topic::waitForMessage<sensor_msgs::JointState>("/motoman/joint_states");
    q << q0->position[0], q0->position[1], q0->position[2], q0->position[3], q0->position[4], q0->position[5], q0->position[6];
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("/command_vel_des", 1, boost::bind(&update_vel, _1, &vipi));
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/motoman/joint_ll_control", 1);
    ros::Publisher cart_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_control", 1);
    ros::Publisher command_vl_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);

    geometry_msgs::TwistStamped cmd_vel_msg;
    

    while (ros::ok() && activate_srv)
    {
        ros::spinOnce();
        cmd_vel_msg.header.stamp = ros::Time::now();
        cmd_vel_msg.twist.linear.x = vipi(0);
        cmd_vel_msg.twist.linear.y = vipi(1);
        cmd_vel_msg.twist.linear.z = vipi(2);
        command_vl_pub.publish(cmd_vel_msg);
        ros::Time start(ros::Time::now().toSec());
        ros::Time t0(ros::Time::now().toSec());
        auto t = t0 - start;

        // get Jacobian
        ROS_INFO_STREAM("vipi: \n"
                        << vipi);

        kinematic_state->setJointGroupPositions(joint_model_group, q);
        kinematic_state->update();
        // while (t.sec < 15)
        // {
        // ros::Time t0(ros::Time::now().toSec());
        // t = t0 - start;

        if (kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian))
        {
            ROS_INFO_STREAM("Jacobian: \n"
                            << jacobian);

            try
            {
                // compute q_dot = pinv(J)*vipi
                q_dot = jacobian.completeOrthogonalDecomposition().solve(vipi);
                // Euler integration
                q = q + euler_sample_time * q_dot;
            
                // send command
                const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();
                sensor_msgs::JointState joint_cmd;
                ROS_INFO_STREAM("Joints value: ");
                for (std::size_t i = 0; i < q.size(); ++i)
                {
                    joint_cmd.name.push_back(joint_names.at(i));
                    joint_cmd.position.push_back(q[i]);
                    joint_cmd.velocity.push_back(q_dot[i]);
                    ROS_INFO_STREAM(q[i]);
                }

                joint_cmd.header.stamp = ros::Time::now();
                joint_cmd_pub.publish(joint_cmd);
            }
            catch (...)
            {
                ROS_ERROR("ALERT: robot command failure");
                return -1;
            }
        }
        else
        {
            ROS_ERROR("Error calculating Jacobian");
            return -1;
        }
        //     loop.sleep();
        // }
        ros::Time end(ros::Time::now().toSec());
        // ROS_INFO_STREAM(end - start);
        loop.sleep();
    }

    return 0;
}

