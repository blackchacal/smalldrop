// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file robot_arm_joint_controller.h Header file for base class robot arm joint controller.
 */

#ifndef _SMALLDROP_ROBOT_ARM_JOINT_CONTROLLER_H
#define _SMALLDROP_ROBOT_ARM_JOINT_CONTROLLER_H

#include <ros/ros.h>

// ROS control
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// ROS message/service
#include <geometry_msgs/Pose.h>
#include <smalldrop_msgs/JointPositions.h>
#include <smalldrop_msgs/Tau.h>

// URDF
#include <urdf/model.h>

// Libraries
#include <Eigen/Dense>
#include <fstream>

namespace smalldrop
{
namespace smalldrop_robot_arm
{
/**
 * \class RobotArmJointController
 * \brief General robot arm joint controller to be use during simulation and with the real robot.
 */
class RobotArmJointController
  : public virtual controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface>
{
  /**
   * \protectedsection
   */
protected:
  /**
   * Class constants
   *****************************************************************************************/

  // Filtering parameters
  //-------------------------------------------------------------------------------------
  const double filter_param_ = 0.1;  /** Defines the frequency divisor for variable filtering. */
  const double delta_tau_max_ = 1.0; /** Defines max torque value sent to the joints. */

  /**
   * Class members
   *****************************************************************************************/

  // Robot joints related members
  //-------------------------------------------------------------------------------------
  unsigned int n_joints_; /** Number of robot joints. */
  std::map<std::string, urdf::JointSharedPtr> robot_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  Eigen::Matrix<double, 7, 1> qdot_, qdot_d_, q_d_, q_d_target_, q_; /** Joint speed, desired joint speed, desired position and position. */

  // Robot joints related members
  //-------------------------------------------------------------------------------------
  Eigen::Matrix<double, 7, 1> error_, vel_error_, error_accum_; /** PID errors. */
  Eigen::Matrix<double, 7, 7> P, P_d, P_d_target;               /** PID proportional gain matrices. */
  Eigen::Matrix<double, 7, 7> D, D_d, D_d_target;               /** PID derivative gain matrices. */
  Eigen::Matrix<double, 7, 7> I, I_d, I_d_target;               /** PID integral gain matrices. */
  double joint1_p_, joint2_p_, joint3_p_, joint4_p_, joint5_p_, joint6_p_,
      joint7_p_; /** Individual joints proportional gains. */
  double joint1_d_, joint2_d_, joint3_d_, joint4_d_, joint5_d_, joint6_d_,
      joint7_d_; /** Individual joints derivative gains. */
  double joint1_i_, joint2_i_, joint3_i_, joint4_i_, joint5_i_, joint6_i_,
      joint7_i_;         /** Individual joints integral gains. */
  double i_clamp_ = 150; /** Clamping limit for integral block. */

  // Joint limits
  //-------------------------------------------------------------------------------------
  Eigen::Matrix<double, 7, 1> min_joint_limits_;      /** \var Vector with all the real minimum joint limits. */
  Eigen::Matrix<double, 7, 1> max_joint_limits_;      /** \var Vector with all the real maximum joint limits. */
  Eigen::Matrix<double, 7, 1> min_user_joint_limits_; /** \var Vector with all the user defined minimum joint limits. */
  Eigen::Matrix<double, 7, 1> max_user_joint_limits_; /** \var Vector with all the user defined maximum joint limits. */

  // Dynamic reconfigure
  //-------------------------------------------------------------------------------------
  ros::NodeHandle dyn_config_gains_node_;

  // Topics & Services
  //-------------------------------------------------------------------------------------
  ros::Subscriber pose_sub_;  /** \var Subscriber to current pose topic. */
  ros::Publisher pose_pub_;   /** \var Publisher to desired pose topic. */
  ros::Publisher joints_pub_; /** \var Publisher to desired joint positions topic. */
  ros::Publisher tau_pub_;    /** \var Publisher to current robot torques topic. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn void updateDynamicConfigGains(void)
   * \brief Updates the PID gains.
   */
  void updateDynamicConfigGains(void);

  /**
   * \fn void filterGains(void)
   * \brief Filter dynamic reconfigure PID gains.
   */
  void filterGains(void);

  /**
   * \fn void setControllerGains(void)
   * \brief Filters the gains, changes to base frame and updates PID matrices.
   */
  void setControllerGains(void);

  /**
   * \fn void saturateIntegralError(void)
   * \brief Saturates the integral error to prevent overdriving the joint actuators.
   */
  void saturateIntegralError(void);

  /**
   * \fn void setupPublishersAndSubscribers(void)
   * \brief Sets up all topics publishing and subscription.
   *
   * \param nh ROS node handle.
   */
  void setupPublishersAndSubscribers(ros::NodeHandle nh);

  /**
   * \fn void updateJointPositionsCallback(const smalldrop_msgs::JointPositions::ConstPtr &msg)
   * \brief Updates the current pose when receiving the desired pose topic.
   *
   * \param msg Desired joint positions topic message.
   */
  void updateJointPositionsCallback(const smalldrop_msgs::JointPositions::ConstPtr &msg);

  /**
   * \fn void publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee)
   * \brief Publishes the robot arm current pose to a topic.
   *
   * \param X0ee End-effector position represented on the base frame.
   * \param R0ee End-effector rotation matrix represented on the base frame.
   */
  void publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee);

  /**
   * \fn void publishCurrentJointPositions(const Eigen::Matrix<double, 7, 1> joints)
   * \brief Publishes the robot arm current joint positions to a topic.
   *
   * \param joints Vector with joint positions.
   */
  void publishCurrentJointPositions(const Eigen::Matrix<double, 7, 1> joints);

  /**
   * \fn void publishTorques(const Eigen::VectorXd tau)
   * \brief Publishes the PID calculated torques (desired torque) to a topic.
   */
  void publishTorques(const Eigen::VectorXd tau);

public:
  RobotArmJointController()
  {
  }
  virtual ~RobotArmJointController()
  {
  }
};

}  // namespace smalldrop_robot_arm
}  // namespace smalldrop

#endif  // _SMALLDROP_ROBOT_ARM_JOINT_CONTROLLER_H