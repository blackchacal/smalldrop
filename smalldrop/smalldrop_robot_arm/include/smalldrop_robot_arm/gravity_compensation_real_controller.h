// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <controller_interface/multi_interface_controller.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

#include <Eigen/Dense>
#include <fstream>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <urdf/model.h>

#include "franka_hw/franka_model_interface.h"
#include "franka_hw/franka_state_interface.h"

namespace smalldrop_robot_arm
{
  class GravityCompensationRealController : public controller_interface::MultiInterfaceController<
                                                    hardware_interface::EffortJointInterface,
                                                    franka_hw::FrankaStateInterface,
                                                    franka_hw::FrankaModelInterface>
  {
  private:
    const double delta_tau_max = 1.0;

    // Robot general variables
    unsigned int n_joints;
    std::vector<hardware_interface::JointHandle> joint_handles;
    std::map<std::string, urdf::JointSharedPtr> robot_joints;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle;

    // Robot Kinematics variables
    Eigen::Matrix4d T0ee;
    Eigen::Vector3d X0ee;
    Eigen::Matrix3d R0ee;
    Eigen::Matrix<double, 7, 1> q, qdot;

    // Robot Dynamics variables
    Eigen::Matrix<double, 6, 7> J;   // Jacobian
    Eigen::Matrix<double, 7, 6> Jhash;  // Dynamically consistent generalized inverse of the jacobian
    Eigen::Matrix<double, 7, 7> M;  // inertia or mass
    Eigen::Matrix<double, 7, 1> C;  // coriolis
    Eigen::Matrix<double, 7, 1> g;  // gravity

    // Publisher/Subscriber
    ros::Publisher posePub;
    ros::Publisher wrenchPub;

    // posture optimization ----------------------------------------------------
    Eigen::Matrix<double, 7, 1> maxJointLimits;
    Eigen::Matrix<double, 7, 1> minJointLimits;

    // Methods
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);
  public:
    bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
    void starting(const ros::Time &time) override;
    void update(const ros::Time &time, const ros::Duration &period) override;
  };
   
} // namespace smalldrop_robot_arm