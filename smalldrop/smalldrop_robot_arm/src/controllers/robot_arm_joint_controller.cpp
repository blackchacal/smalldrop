// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file robot_arm_joint_controller.cpp
 */

#include <smalldrop_robot_arm/robot_arm_joint_controller.h>

namespace smalldrop
{
namespace smalldrop_robot_arm
{
/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

/**
 * \brief Sets up all topics publishing and subscription.
 */
void RobotArmJointController::setupPublishersAndSubscribers(ros::NodeHandle nh)
{
  pose_sub_ = nh.subscribe("/smalldrop/robot_arm/desired_joint_positions", 10, &RobotArmJointController::updateJointPositionsCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::Pose>("/smalldrop/robot_arm/current_pose", 10);
  joints_pub_ = nh.advertise<smalldrop_msgs::JointPositions>("/smalldrop/robot_arm/current_joint_positions", 10);
  tau_pub_ = nh.advertise<smalldrop_msgs::Tau>("/smalldrop/robot_arm/tau", 10);
}

/**
 * \brief Updates the current joint configuration when receiving the desired joint positions topic.
 */
void RobotArmJointController::updateJointPositionsCallback(const smalldrop_msgs::JointPositions::ConstPtr &msg)
{
  for (size_t i = 0; i < n_joints_; i++)
  {
    if (msg->positions[i] > max_user_joint_limits_(i))
      q_d_target_(i) = max_user_joint_limits_(i);
    else
      q_d_target_(i) = msg->positions[i];

    if (msg->positions[i] < min_user_joint_limits_(i))
      q_d_target_(i) = min_user_joint_limits_(i);
    else
      q_d_target_(i) = msg->positions[i];
  }
}

/**
 * \brief Publishes the robot arm current pose to a topic.
 */
void RobotArmJointController::publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee)
{
  geometry_msgs::Pose msg;
  msg.position.x = X0ee[0];
  msg.position.y = X0ee[1];
  msg.position.z = X0ee[2];
  Eigen::Quaterniond quat(R0ee);
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();
  pose_pub_.publish(msg);
}

/**
 * \copybrief RobotArmJointController::publishCurrentJointPositions()
 */
void RobotArmJointController::publishCurrentJointPositions(const Eigen::Matrix<double, 7, 1> joints)
{
  smalldrop_msgs::JointPositions msg;
  for (size_t i = 0; i < joints.size(); i++)
  {
    msg.names.push_back(joint_handles_[i].getName());
    msg.positions.push_back(joints(i));
  }

  joints_pub_.publish(msg);
}

/**
 * \brief Publishes the PID calculated torques (desired torque) to a topic.
 */
void RobotArmJointController::publishTorques(const Eigen::VectorXd tau)
{
  smalldrop_msgs::Tau tau_msg;
  tau_msg.joint_name.resize(joint_handles_.size());

  // Prepare Tau msg
  for (size_t i = 0; i < joint_handles_.size(); i++)
    tau_msg.joint_name[i] = joint_handles_[i].getName();

  tau_msg.tau_d.resize(tau.size());
  Eigen::VectorXd::Map(&tau_msg.tau_d[0], tau.size()) = tau;

  tau_pub_.publish(tau_msg);
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \brief Updates the impedance gains.
 *
 * It builds the target PID gains matrices for PID control. They will be filtered
 * at controller update to avoid robot jerks when the updated value is very different from the previous.
 */
void RobotArmJointController::updateDynamicConfigGains(void)
{
  // Proportional gain
  P_d_target << joint1_p_, 0, 0, 0, 0, 0, 0,
                0, joint2_p_, 0, 0, 0, 0, 0,
                0, 0, joint3_p_, 0, 0, 0, 0,
                0, 0, 0, joint4_p_, 0, 0, 0,
                0, 0, 0, 0, joint5_p_, 0, 0,
                0, 0, 0, 0, 0, joint6_p_, 0,
                0, 0, 0, 0, 0, 0, joint7_p_;

  // Derivative gain
  D_d_target << joint1_d_, 0, 0, 0, 0, 0, 0,
                0, joint2_d_, 0, 0, 0, 0, 0,
                0, 0, joint3_d_, 0, 0, 0, 0,
                0, 0, 0, joint4_d_, 0, 0, 0,
                0, 0, 0, 0, joint5_d_, 0, 0,
                0, 0, 0, 0, 0, joint6_d_, 0,
                0, 0, 0, 0, 0, 0, joint7_d_;

  // Integral gains
  I_d_target << joint1_i_, 0, 0, 0, 0, 0, 0,
                0, joint2_i_, 0, 0, 0, 0, 0,
                0, 0, joint3_i_, 0, 0, 0, 0,
                0, 0, 0, joint4_i_, 0, 0, 0,
                0, 0, 0, 0, joint5_i_, 0, 0,
                0, 0, 0, 0, 0, joint6_i_, 0,
                0, 0, 0, 0, 0, 0, joint7_i_;
}

/**
 * \fn void filterGains(void)
 * \brief Filter dynamic reconfigure impedance gains.
 */
void RobotArmJointController::filterGains(void)
{
  P_d = filter_param_ * P_d_target + (1.0 - filter_param_) * P_d;
  D_d = filter_param_ * D_d_target + (1.0 - filter_param_) * D_d;
  I_d = filter_param_ * I_d_target + (1.0 - filter_param_) * I_d;
}

/**
 * \fn void setControllerGains(void)
 * \brief Filters the gains, changes to base frame and updates impedance matrices.
 */
void RobotArmJointController::setControllerGains(void)
{
  // Filter the gains so that the response is not abrupt!
  filterGains();

  P.setIdentity();
  P << P_d;

  D.setIdentity();
  D << D_d;

  I.setIdentity();
  I << I_d;
}

/**
 * \brief Saturates the integral error to prevent overdriving the joint actuators.
 *
 * If the error goes above positive clamping limit or below negative clamping limit
 * it should use the respective limit values.
 */
void RobotArmJointController::saturateIntegralError(void)
{
  for (size_t i = 0; i < n_joints_; i++)
  {
    if (error_accum_[i] > i_clamp_)
      error_accum_[i] = i_clamp_;
    else if (error_accum_[i] < -i_clamp_)
      error_accum_[i] = -i_clamp_;
  }
}

}  // namespace smalldrop_robot_arm

}  // namespace smalldrop