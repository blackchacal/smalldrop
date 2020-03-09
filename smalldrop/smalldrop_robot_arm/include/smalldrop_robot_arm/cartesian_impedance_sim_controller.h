// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file cartesian_impedance_sim_controller.h Header file for class cartesiam impedance simulation controller.
 */

#ifndef _SMALLDROP_CARTESIAN_IMPEDANCE_SIM_CONTROLLER_H
#define _SMALLDROP_CARTESIAN_IMPEDANCE_SIM_CONTROLLER_H

#include <smalldrop_robot_arm/robot_arm_controller.h>

// Dynamic reconfigure
#include <smalldrop_robot_arm/CartesianImpedanceSimControllerConfig.h>

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
 * \class CartesianImpedanceSimController
 * \brief Cartesian impedance controller to be use during simulation.
 */
class CartesianImpedanceSimController : public virtual RobotArmController
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
  std::unique_ptr<dynamic_reconfigure::Server<::smalldrop_robot_arm::CartesianImpedanceSimControllerConfig>>
      dyn_config_gains_param_;

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn bool calcJacobian(KDL::Jacobian &J, const Eigen::Matrix<double, 7, 1> &q_in)
   * \brief Calculate the Jacobian matrix using KDL library.
   * 
   * \param J Jacobian matrix in KDL format.
   * \param q_in Joint positions.
   */
  bool calcJacobian(KDL::Jacobian &J, const Eigen::Matrix<double, 7, 1> &q_in);

  /**
   * \fn bool jacobian(Eigen::Matrix<double, 6, 7> &J_out, const Eigen::Matrix<double, 7, 1> &q_in)
   * \brief Transforms the KDL format Jacobian to an Eigen matrix.
   * 
   * \param J_out Jacobian matrix in Eigen format (output).
   * \param q_in Joint positions.
   */
  bool jacobian(Eigen::Matrix<double, 6, 7> &J_out, const Eigen::Matrix<double, 7, 1> &q_in);

  /**
   * \fn bool fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf)
   * \brief Calculates forward kinematics using the KDL library.
   * 
   * \param q_in Joint positions.
   * \param transf Homogeneous transformation between the end-effector and base link (output).
   */
  bool fk(const Eigen::Matrix<double, 7, 1> &q_in, Eigen::Matrix4d &transf);

  /**
   * \fn bool dynamic(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &qdot)
   * \brief Calculate robot dynamics using KDL library. The dynamic class members are updated inside the method.
   * 
   * \param q Joint positions.
   * \param qdot Joint velocities.
   */
  bool dynamic(Eigen::Matrix<double, 7, 1> &q, Eigen::Matrix<double, 7, 1> &qdot);

  /**
   * \fn void updateDynamicConfigGainsCallback(smalldrop_robot_arm::CartesianImpedanceSimControllerConfig &config, uint32_t level)
   * \brief Updates the impedance gains using the dynamic reconfigure for simulation.
   * 
   * \param config Dynamic reconfigure config instance.
   * \param level Dynamic reconfigure level.
   */
  void updateDynamicConfigGainsCallback(::smalldrop_robot_arm::CartesianImpedanceSimControllerConfig &config, uint32_t level);

  /**
   * \fn bool sendTorquesToRobot(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
   * \brief Service callback to set if calculated torques should be sent to the robot.
   * 
   * \param req Service request object.
   * \param res Service response object.
   */
  bool sendTorquesToRobot(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

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

#endif // _SMALLDROP_CARTESIAN_IMPEDANCE_SIM_CONTROLLER_H
