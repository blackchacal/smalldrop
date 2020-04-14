// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file robot_arm_controller.h Header file for base class robot arm controller.
 */

#ifndef _SMALLDROP_ROBOT_ARM_CONTROLLER_H
#define _SMALLDROP_ROBOT_ARM_CONTROLLER_H

#include <ros/ros.h>

// ROS control
#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>

// URDF
#include <urdf/model.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>

// ROS message/service
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Wrench.h>
#include <smalldrop_msgs/Tau.h>
#include <smalldrop_msgs/TrackingError.h>
#include "std_srvs/SetBool.h"

// Libraries
#include <Eigen/Dense>
#include <fstream>

namespace smalldrop
{
namespace smalldrop_robot_arm
{

/**
 * \class RobotArmController
 * \brief General robot arm controller to be use during simulation and with the real robot.
 */
class RobotArmController
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
  bool torques_to_robot_ = true; /** \var Flag the controls if the calculated torques are sent to the robot joints or not. */

  double i_clamp_p_ = 150; /** Clamping limit for impedance integral block on position. */
  double i_clamp_o_ = 100; /** Clamping limit for impedance integral block on orientation. */

  // Robot joints related members
  //-------------------------------------------------------------------------------------
  unsigned int n_joints_;                                       /** \var Number of robot joints. */
  std::map<std::string, urdf::JointSharedPtr> robot_joints_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  Eigen::Matrix<double, 7, 1> min_joint_limits_;                /** \var Vector with all the minimum joint limits. */
  Eigen::Matrix<double, 7, 1> max_joint_limits_;                /** \var Vector with all the maximum joint limits. */
  Eigen::Matrix<double, 7, 1> nullspace_objective_;             /** \var Null space objective. */

  // Robot Kinematics members
  //-------------------------------------------------------------------------------------
  Eigen::Matrix4d T0ee_;                      /** \var Current homogeneous transformation between the end-effector and the base link. */
  Eigen::Matrix4d T0ee_d_;                    /** \var Desired homogeneous transformation between the end-effector and the base link. */
  Eigen::Matrix<double, 7, 1> q_;             /** \var Current joint positions. */
  Eigen::Matrix<double, 7, 1> qdot_;          /** \var Current joint velocities */
  Eigen::Vector3d X0ee_d_;                    /** \var Desired end-effector position represented on base frame. */
  Eigen::Vector3d X0ee_d_target_;             /** \var Desired end-effector position filtered, represented on base frame. */
  Eigen::Vector3d X0ee_d_prev_;               /** \var Previous desired end-effector position represented on base frame. */
  Eigen::Matrix3d R0ee_d_;                    /** \var Desired end-effector orientation matrix represented on base frame. */
  Eigen::Matrix3d R0ee_d_target_;             /** \var Desired end-effector orientation matrix filtered, represented on base frame. */
  Eigen::Quaterniond orient_d_;               /** \var Desired end-effector orientation quaternion represented on base frame. */
  Eigen::Quaterniond orient_d_target_;        /** \var Desired end-effector orientation quaternion filtered, represented on base frame. */
  Eigen::Matrix<double, 6, 1> vel_d_;         /** \var Desired end-effector velocity. */
  Eigen::Matrix<double, 6, 1> error_;         /** \var Current end-effector error. */
  Eigen::Matrix<double, 6, 1> error_prev_;    /** \var Previous end-effector error. */
  Eigen::Matrix<double, 6, 1> error_accum_;   /** \var Accumulated end-effector error for impedance integral block. */
  Eigen::Matrix<double, 6, 1> vel_error_;     /** \var Current end-effector velocity error. */

  // Robot Dynamics members
  //-------------------------------------------------------------------------------------
  Eigen::Matrix<double, 7, 1> tau_;             /** \var Current joint torques. */
  Eigen::Matrix<double, 7, 1> tau_initial_;     /** \var Initial joint torques obtained during controller start. */
  Eigen::Matrix<double, 6, 7> J;                /** \var Jacobian matrix. */
  Eigen::Matrix<double, 7, 6> Jhash;            /** \var Dynamically consistent generalized inverse of the jacobian. */
  Eigen::Matrix<double, 7, 7> M;                /** \var Inertia/Mass Matrix. */
  Eigen::Matrix<double, 7, 1> C;                /** \var Coriolis vector. */
  Eigen::Matrix<double, 7, 1> g;                /** \var Gravity vector. */
  Eigen::Matrix<double, 6, 6> cart_K;           /** \var Cartesian stiffness matrix for impedance control. */
  Eigen::Matrix<double, 6, 6> cart_D;           /** \var Cartesian damping matrix for impedance control. */
  Eigen::Matrix<double, 6, 6> cart_I;           /** \var Cartesian integral matrix for impedance control. */
  Eigen::Matrix<double, 7, 7> null_K;           /** \var Actual nullspace stiffness matrix. */
  Eigen::Matrix<double, 7, 7> null_K_d;         /** \var Desired nullspace stiffness matrix. */
  Eigen::Matrix<double, 7, 7> null_K_d_target;  /** \var Desired nullspace stiffness matrix filtered. */
  
  Eigen::Matrix3d Kp_d, Dp_d, Ip_d, Kp_d_target, Dp_d_target, Ip_d_target;  // position stiffness, damping and integral in desired frame
  Eigen::Matrix3d Ko_d, Do_d, Io_d, Ko_d_target, Do_d_target, Io_d_target;  // orientation stiffness, damping and integral in desired frame
  double Kpx, Kpy, Kpz, Kox, Koy, Koz, Dpx, Dpy, Dpz, Dox, Doy, Doz, Ipx, Ipy, Ipz, Iox, Ioy, Ioz, Kpn;

  // Workspace limits
  //-------------------------------------------------------------------------------------
  double wsp_x_min_limit_; /** \var Workspace minimum limit on the x axis. */
  double wsp_x_max_limit_; /** \var Workspace maximum limit on the x axis. */
  double wsp_y_min_limit_; /** \var Workspace minimum limit on the y axis. */
  double wsp_y_max_limit_; /** \var Workspace maximum limit on the y axis. */
  double wsp_z_min_limit_; /** \var Workspace minimum limit on the z axis. */
  double wsp_z_max_limit_; /** \var Workspace maximum limit on the z axis. */

  // Dynamic reconfigure
  //-------------------------------------------------------------------------------------
  ros::NodeHandle dyn_config_gains_node_;

  // Topics & Services
  //-------------------------------------------------------------------------------------
  ros::Subscriber pose_sub_;        /** \var Subscriber to current pose topic. */
  ros::Publisher pose_pub_;         /** \var Publisher to desired pose topic. */
  ros::Publisher tau_pub_;          /** \var Publisher to current robot torques topic. */
  ros::Publisher wrench_pub_;       /** \var Publisher to current robot wrenches topic. */
  ros::Publisher error_pub_;        /** \var Publisher to current position and orientation errors topic. */
  ros::ServiceServer control_srv_;  /** \var Service server to control sendind the robot torques to the robot externally. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn Eigen::Vector3d R2r(Eigen::Matrix3d &rotation)
   * \brief Computes the axis/angle representation from a rotation matrix.
   * 
   * \param Rotation Rotation matrix.
   * 
   * \return Returns a 3D vector (double).
   */
  Eigen::Vector3d R2r(Eigen::Matrix3d &rotation);

  /**
   * \fn void updateDynamicConfigGains(void)
   * \brief Updates the impedance gains.
   */
  void updateDynamicConfigGains(void);

  /**
   * \fn double calcJointDerivative(const double q_i, const double min_joint_limit_i, const double max_joint_limit_i)
   * \brief Calculate the joint derivative between its limits.
   * 
   * \param q_i Joint i position.
   * \param min_joint_limit_i Joint i minimum limit.
   * \param max_joint_limit_i Joint i maximum limit.
   */
  double calcJointDerivative(const double q_i, const double min_joint_limit_i, const double max_joint_limit_i);
  
  /**
   * \fn Eigen::Matrix<double, 7, 1> distanceFromMechanicalJointLimits(const Eigen::Matrix<double, 7, 1> q, const Eigen::Matrix<double, 7, 1> min_joint_limits, const Eigen::Matrix<double, 7, 1> max_joint_limits)
   * \brief Calculates distance from mechanical joint limits null space objective function.
   * 
   * \param q Vector with joint positions.
   * \param min_joint_limits Vector with joint minimum limits.
   * \param max_joint_limits Vector with joint maximum limits.
   */
  Eigen::Matrix<double, 7, 1> distanceFromMechanicalJointLimits(const Eigen::Matrix<double, 7, 1> q, 
                                                                const Eigen::Matrix<double, 7, 1> min_joint_limits, 
                                                                const Eigen::Matrix<double, 7, 1> max_joint_limits);
  
  /**
   * \fn void nullSpaceObjectiveFunction(Eigen::Matrix<double, N, 1> &objective)
   * \brief Calculates null space objective function.
   * 
   * \param objective Objective result as output.
   */
  void nullSpaceObjectiveFunction(Eigen::Matrix<double, 7, 1> &objective);

  /**
   * \fn void filterGains(void)
   * \brief Filter dynamic reconfigure impedance gains. 
   */
  void filterGains(void);

  /**
   * \fn void setControllerGains(void)
   * \brief Filters the gains, changes to base frame and updates impedance matrices. 
   */
  void setControllerGains(void);

  /**
   * \fn void calcIntegralError(void)
   * \brief Calculates the accumulated end-effector error (position & orientation), over 
   * time for the impedance integral block.
   */
  void calcIntegralError(void);

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
   * \fn void updatePoseCallback(const geometry_msgs::PoseConstPtr &msg)
   * \brief Updates the current pose when receiving the desired pose topic.
   * 
   * \param msg Desired pose topic message.
   */
  void updatePoseCallback(const geometry_msgs::PoseConstPtr &msg);

  /**
   * \fn void publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee)
   * \brief Publishes the robot arm current pose to a topic.
   * 
   * \param X0ee End-effector position represented on the base frame. 
   * \param R0ee End-effector rotation matrix represented on the base frame. 
   */
  void publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee);

  /**
   * \fn void publishTorques(const Eigen::VectorXd tau_d, const Eigen::VectorXd tau_task, const Eigen::VectorXd tau_null)
   * \brief Publishes the impedance calculated torques (desired torque, task torque and null space torque) to a topic.
   */
  void publishTorques(const Eigen::VectorXd tau_d, const Eigen::VectorXd tau_task, const Eigen::VectorXd tau_null);

  /**
   * \fn void publishWrenches(void)
   * \brief Publishes the robot external wrenches to a topic.
   */
  virtual void publishWrenches(void);

  /**
   * \fn void publishTrackingErrors(void)
   * \brief Publishes the robot tracking error to a topic.
   */
  void publishTrackingErrors(void);

public:
  RobotArmController() {}
  virtual ~RobotArmController() {}
};

}  // namespace smalldrop_robot_arm
}  // namespace smalldrop

#endif // _SMALLDROP_ROBOT_ARM_CONTROLLER_H