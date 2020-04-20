// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file joint_position_pid_sim_controller.h Header file for class robot arm joint pid controller.
 */

#ifndef _SMALLDROP_ROBOT_ARM_JOINT_POSITION_PID_SIM_CONTROLLER_H
#define _SMALLDROP_ROBOT_ARM_JOINT_POSITION_PID_SIM_CONTROLLER_H

#include <smalldrop_robot_arm/robot_arm_joint_controller.h>

// Dynamic reconfigure
#include <smalldrop_robot_arm/JointPositionPIDSimControllerConfig.h>

// KDL library
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/solveri.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace smalldrop
{
namespace smalldrop_robot_arm
{
/**
 * \class JointPositionPIDSimController
 * \brief Robot arm joint position PID controller to be use during simulation and with the real robot.
 */
class JointPositionPIDSimController : public RobotArmJointController
{
private:
  /**
   * Class members
   *****************************************************************************************/

  // KDL library related members
  //-------------------------------------------------------------------------------------
  KDL::Tree k_tree_;    /** \var Kinematic tree obtained from URDF model. */
  KDL::Chain k_chain_;  /** \var Kinematic chain. */

  // Dynamic reconfigure
  //-------------------------------------------------------------------------------------
  std::unique_ptr<dynamic_reconfigure::Server<::smalldrop_robot_arm::JointPositionPIDSimControllerConfig>>
      dyn_config_gains_param_;

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn bool fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf)
   * \brief Calculates forward kinematics using the KDL library.
   * 
   * \param q_in Joint positions.
   * \param transf Homogeneous transformation between the end-effector and base link (output).
   */
  bool fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf);

  /**
   * \fn void updateDynamicConfigGainsCallback(::smalldrop_robot_arm::JointPositionPIDSimControllerConfig &config, uint32_t level)
   * \brief Updates the impedance gains using the dynamic reconfigure for simulation.
   * 
   * \param config Dynamic reconfigure config instance.
   * \param level Dynamic reconfigure level.
   */
  void updateDynamicConfigGainsCallback(::smalldrop_robot_arm::JointPositionPIDSimControllerConfig &config, uint32_t level);

public:
  /**
   * Class methods
   *****************************************************************************************/

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
};

}  // namespace smalldrop_robot_arm

}  // namespace smalldrop

#endif  // _SMALLDROP_ROBOT_ARM_JOINT_POSITION_PID_SIM_CONTROLLER_H