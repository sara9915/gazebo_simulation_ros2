#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "uclv_moveit_planner_ros2/planner.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TwistCmd : public rclcpp::Node
{
public:
  TwistCmd()
      : Node("vel_cmd_node")
  {
    // Euler variables initialization
    std::cout << BOLDBLUE << "Initializing Euler variables..." << RESET << std::endl;
    this->vipi << 0, 0, 0, 0, 0, 0;
    this->q_dot << 0, 0, 0, 0, 0, 0, 0;
    this->q << 0, 0, 0, 0, 0, 0, 0;
    this->reference_point_position << 0.0, 0.0, 0.0;

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "execute_traj_action_server");
    while (!parameters_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    this->simulation = parameters_client->get_parameters({"simulation"}).at(0).as_bool();
    std::cout << BOLDWHITE << "Simulation: " << RESET << this->simulation << std::endl;

    std::thread([this]()
                {
                  this->get_clock()->sleep_for(std::chrono::seconds(2));
                  std::cout << BOLDBLUE << "Creating MoveIt object..." << RESET << std::endl;
                  planner_moveit_ = std::make_shared<uclv::PlannerMoveIt>(this->shared_from_this(), "yaskawa_arm");
                  
                  // Initialize joint values
                  auto current_joints = planner_moveit_->move_group.getCurrentJointValues();
                  this->q << current_joints.at(0), current_joints.at(1), current_joints.at(2), current_joints.at(3), current_joints.at(4), current_joints.at(5), current_joints.at(6);
                  std::cout << BOLDBLUE << "Get current joints value q: " << RESET << this->q << std::endl;

                  // Initialize publishers to publish the reference joint states and the reference twist to be actuated
                  if(this->simulation)
                  {
                    trajectory_joints_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/yaskawa_arm_controller/joint_trajectory", 1);
                    std::cout << BOLDGREEN << "Creating the publisher for simulation... " << RESET << std::endl;
                  }
                  else
                  {
                    joints_ref_pub = this->create_publisher<sensor_msgs::msg::JointState>("/motoman/joint_ll_control", 1); 
                    std::cout << BOLDGREEN << "Creating the publisher for robot... " << RESET << std::endl;

                  }

                  twist_ref_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/twist_ref", 1);

                  // Initialize subscriber to read the twist command from the controller
                  twist_ref_sub = this->create_subscription<geometry_msgs::msg::Twist>("/twist_controller", 1, std::bind(&TwistCmd::update_vel, this, _1));
                  
                  // Initialize timer to publish the reference joint states and the reference twist to be actuated
                  timer_ = this->create_wall_timer(
                      std::chrono::duration<double>(euler_sample_time), std::bind(&TwistCmd::timer_callback, this)); })
        .detach();
  }

private:
  // Callbacks
  void update_vel(const geometry_msgs::msg::Twist &msg)
  {
    this->vipi << msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z; // desired pusher velocity in world frame
  }

  void timer_callback()
  {
    geometry_msgs::msg::TwistStamped twist_ref_msg;

    twist_ref_msg.header.stamp = this->now();
    twist_ref_msg.header.frame_id = "world";
    twist_ref_msg.twist.linear.x = this->vipi(0);
    twist_ref_msg.twist.linear.y = this->vipi(1);
    twist_ref_msg.twist.linear.z = this->vipi(2);
    twist_ref_msg.twist.angular.x = this->vipi(3);
    twist_ref_msg.twist.angular.y = this->vipi(4);
    twist_ref_msg.twist.angular.z = this->vipi(5);

    // Publishing the reference twist to be actuated
    std::cout << BOLDGREEN << "Publishing the reference twist to be actuated: " << RESET << std::endl;
    twist_ref_pub->publish(twist_ref_msg);

    // Get Jacobian
    planner_moveit_->start_state->setJointGroupPositions(planner_moveit_->joint_model_group, this->q);
    planner_moveit_->start_state->update();

    if (planner_moveit_->start_state->getJacobian(planner_moveit_->joint_model_group, planner_moveit_->start_state->getLinkModel(planner_moveit_->joint_model_group->getLinkModelNames().back()), this->reference_point_position, this->jacobian))
    {
      std::cout << BOLDGREEN << "Jacobian: \n"
                << RESET << std::endl;
      RCLCPP_INFO_STREAM(this->get_logger(), jacobian);

      try
      {
        // compute q_dot = pinv(J)*vipi
        this->q_dot = jacobian.completeOrthogonalDecomposition().solve(this->vipi);
        // Euler integration
        this->q = this->q + this->euler_sample_time * this->q_dot;

        // SEND COMMAND
        const std::vector<std::string> &joint_names = planner_moveit_->joint_model_group->getVariableNames();

        if (this->simulation)
        {
          std::cout << BOLDGREEN << "Creating joint trajectory msg to simulate.." << RESET << std::endl;
          trajectory_msgs::msg::JointTrajectory trajectory_joints_msg;
          trajectory_joints_msg.points.resize(1);
          trajectory_joints_msg.header.stamp = this->now() + std::chrono::duration<double>(0.01);
          for (int i = 0; i < int(q.size()); ++i)
          {
            trajectory_joints_msg.joint_names.push_back(joint_names.at(i));
            trajectory_joints_msg.points.at(0).positions.push_back(q[i]);
            trajectory_joints_msg.points[0].velocities.push_back(q_dot[i]);
            RCLCPP_INFO_STREAM(this->get_logger(), q[i]);
          }

          // Publishing the reference joint states to be actuated
          std::cout << BOLDGREEN << "Publishing the reference joint states to be actuated: " << RESET << std::endl;
          trajectory_joints_pub->publish(trajectory_joints_msg);
        }
        else
        {
          sensor_msgs::msg::JointState joints_ref_msg;
          joints_ref_msg.header.stamp = this->now();
          for (int i = 0; i < int(q.size()); ++i)
          {
            joints_ref_msg.name.push_back(joint_names.at(i));
            joints_ref_msg.position.push_back(q[i]);
            joints_ref_msg.velocity.push_back(q_dot[i]);
            RCLCPP_INFO_STREAM(this->get_logger(), q[i]);
          }

          // Publishing the reference joint states to be actuated
          std::cout << BOLDGREEN << "Publishing the reference joint states to be actuated: " << RESET << std::endl;
          joints_ref_pub->publish(joints_ref_msg);
        }
      }
      catch (...)
      {
        RCLCPP_ERROR(this->get_logger(), "ALERT: robot command failure");
        return;
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "ALERT: Error evaluating the Jacobian");
      return;
    }
  }

  // Publisher and subscriber
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_ref_pub;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_joints_pub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_ref_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_ref_sub;
  rclcpp::TimerBase::SharedPtr timer_;

  // MoveIt variables (used to calculate Jacobian)
  std::shared_ptr<uclv::PlannerMoveIt> planner_moveit_;

  // Usefull variables for Euler integration
  Eigen::Matrix<double, 6, 1> vipi;
  Eigen::Matrix<double, 7, 1> q_dot;
  Eigen::Matrix<double, 7, 1> q;
  Eigen::MatrixXd jacobian;
  Eigen::Vector3d reference_point_position;

  double euler_frequency = 40; // [Hz]
  double euler_sample_time = 1 / euler_frequency;
  double simulation;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistCmd>());
  rclcpp::shutdown();
  return 0;
}