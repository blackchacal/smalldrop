// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file cartesian_impedance_real_controller.h Header file for class cartesiam impedance real controller.
 */

#ifndef _SMALLDROP_CARTESIAN_IMPEDANCE_REAL_CONTROLLER_H
#define _SMALLDROP_CARTESIAN_IMPEDANCE_REAL_CONTROLLER_H

#include <smalldrop_robot_arm/robot_arm_controller.h>

// Dynamic reconfigure
#include <smalldrop_robot_arm/CartesianImpedanceRealControllerConfig.h>

// Franka Hardware
#include "franka_hw/franka_model_interface.h"
#include "franka_hw/franka_state_interface.h"

namespace smalldrop_robot_arm
{
/**
 * \class CartesianImpedanceRealController
 * \brief Cartesian impedance controller to be use during real robot operation.
 */
class CartesianImpedanceRealController : public virtual RobotArmController
{
  /**
   * Class members
   *****************************************************************************************/

  // Franka robot state and model members
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<smalldrop_robot_arm::CartesianImpedanceRealControllerConfig>>
      dyn_config_gains_param_;

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d_calculated, const Eigen::Matrix<double, 7, 1> &tau_J_d) 
   * \brief Clamps the joint torques if they go beyond the defined limits.
   *
   * \param tau_d Calculated desired torque vector.
   * \param tau_J_d Desired link-side joint torque sensor signals without gravity.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1> &tau_d,
                                                 const Eigen::Matrix<double, 7, 1> &tau_J_d);

  /**
   * \fn void updateDynamicConfigGainsCallback(smalldrop_robot_arm::CartesianImpedanceRealControllerConfig &config, uint32_t level) 
   * \brief Updates the impedance gains using the dynamic reconfigure for the real robot.
   *
   * \param config Dynamic reconfigure config instance.
   * \param level Dynamic reconfigure level.
   */
  void updateDynamicConfigGainsCallback(smalldrop_robot_arm::CartesianImpedanceRealControllerConfig &config,
                                        uint32_t level);

  /**
   * \fn void publishWrenches(void)
   * \brief Publishes the robot external wrenches to a topic.
   */
  void publishWrenches(void) override;                                      

public:
  /**
   * Class methods
   *****************************************************************************************/

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &nh) override;
  void starting(const ros::Time &time) override;
  void update(const ros::Time &time, const ros::Duration &period) override;
};

}  // namespace smalldrop_robot_arm

#endif  // _SMALLDROP_CARTESIAN_IMPEDANCE_REAL_CONTROLLER_H