// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file robot_arm_controller.cpp
 */

#include <smalldrop_robot_arm/robot_arm_controller.h>

namespace smalldrop_robot_arm
{
/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

/**
 * \brief Sets up all topics publishing and subscription.
 */
void RobotArmController::setupPublishersAndSubscribers(ros::NodeHandle nh)
{
  pose_sub_ = nh.subscribe("/smalldrop/robot_arm/desired_pose", 10, &RobotArmController::updatePoseCallback, this);
  pose_pub_ = nh.advertise<geometry_msgs::Pose>("/smalldrop/robot_arm/current_pose", 10);
  tau_pub_ = nh.advertise<smalldrop_msgs::Tau>("/smalldrop/robot_arm/tau", 10);
  wrench_pub_ = nh.advertise<geometry_msgs::Wrench>("/smalldrop/robot_arm/wrench", 10);
  error_pub_ = nh.advertise<smalldrop_msgs::TrackingError>("/smalldrop/robot_arm/error", 10);
}

/**
 * \brief Updates the current pose when receiving the desired pose topic.
 *
 * Passes the position components from the message directly to the target position variabe.
 * The orientation part (quaternion) needs to be checked for signal change and updated accordingly before
 * setting it as desired orientation. This prevents the end-effector to flip directions and accelerate to
 * update the orientation.
 */
void RobotArmController::updatePoseCallback(const geometry_msgs::PoseConstPtr &msg)
{
  X0ee_d_target_ << msg->position.x, msg->position.y, msg->position.z;

  Eigen::Quaterniond last_orient_d_target_(orient_d_target_);
  orient_d_target_.coeffs() << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

  if (last_orient_d_target_.coeffs().dot(orient_d_target_.coeffs()) < 0.0)
    orient_d_target_.coeffs() << -orient_d_target_.coeffs();
}

/**
 * \brief Publishes the robot arm current pose to a topic.
 */
void RobotArmController::publishCurrentPose(const Eigen::Vector3d X0ee, const Eigen::Matrix3d R0ee)
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
 * \brief Publishes the impedance calculated torques (desired torque, task torque and null space torque) to a topic.
 */
void RobotArmController::publishTorques(const Eigen::VectorXd tau_d, const Eigen::VectorXd tau_task,
                                        const Eigen::VectorXd tau_null)
{
  smalldrop_msgs::Tau tau_msg;
  tau_msg.joint_name.resize(joint_handles_.size());

  // Prepare Tau msg
  for (size_t i = 0; i < joint_handles_.size(); i++)
    tau_msg.joint_name[i] = joint_handles_[i].getName();

  tau_msg.tau_d.resize(tau_d.size());
  tau_msg.tau_task.resize(tau_task.size());
  tau_msg.tau_null.resize(tau_null.size());
  Eigen::VectorXd::Map(&tau_msg.tau_d[0], tau_d.size()) = tau_d;
  Eigen::VectorXd::Map(&tau_msg.tau_task[0], tau_task.size()) = tau_task;
  Eigen::VectorXd::Map(&tau_msg.tau_null[0], tau_null.size()) = tau_null;

  tau_pub_.publish(tau_msg);
}

/**
 * \brief Publishes the robot external wrenches to a topic.
 */
void RobotArmController::publishWrenches(void)
{
  geometry_msgs::Wrench wrench_msg;

  // Prepare force msg
  Eigen::VectorXd wrench = Eigen::VectorXd(6);
  wrench = Jhash.transpose() * (tau_ - tau_initial_);
  wrench_msg.force.x = wrench[0];
  wrench_msg.force.y = wrench[1];
  wrench_msg.force.z = wrench[2];
  wrench_msg.torque.x = wrench[3];
  wrench_msg.torque.y = wrench[4];
  wrench_msg.torque.z = wrench[5];

  wrench_pub_.publish(wrench_msg);
}

/**
 * \brief Publishes the robot tracking error to a topic.
 */
void RobotArmController::publishTrackingErrors(void)
{
  smalldrop_msgs::TrackingError error_msg;
  error_msg.error.resize(6);
  // Prepare tracking error msg
  Eigen::VectorXd::Map(&error_msg.error[0], error_.size()) = error_;
  error_pub_.publish(error_msg);
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \brief Computes the axis/angle representation from a rotation matrix.
 *
 * Uses a simplified version of the conversion of a rotation matrix to axis/angle representation.
 * Instead of having r = theta * k, where theta is uses an arccos() and K uses a sin(theta),
 * the calculation used is new_r = sin(theta) * k.
 *
 * NOTE: This can only be used for small rotation angles where sin(theta) ~ theta.
 */
Eigen::Vector3d RobotArmController::R2r(Eigen::Matrix3d &rotation)
{
  Eigen::Vector3d r; /** \var Axis/Angle rotation vector. */
  Eigen::Vector3d aux;

  aux << rotation(2, 1) - rotation(1, 2),  // (r32 - r23)
      rotation(0, 2) - rotation(2, 0),     // (r13 - r31)
      rotation(1, 0) - rotation(0, 1);     // (r21 - r12)

  // The traditional formula would be
  // r = 1 / (2*sin(theta)) * aux
  r = 0.5 * aux;

  return r;
}

/**
 * \brief Updates the impedance gains.
 *
 * It builds the target impedance gains matrices for impedance control. They will be filtered
 * at controller update to avoid robot jerks when the updated value is very different from the previous.
 */
void RobotArmController::updateDynamicConfigGains(void)
{
  // position stiffness in desired frame
  Kp_d_target << Kpx, 0, 0, 0, Kpy, 0, 0, 0, Kpz;

  // orientation stiffness in desired frame
  Ko_d_target << Kox, 0, 0, 0, Koy, 0, 0, 0, Koz;

  // position damping in desired frame
  Dp_d_target << Dpx, 0, 0, 0, Dpy, 0, 0, 0, Dpz;

  // orientation damping in desired frame
  Do_d_target << Dox, 0, 0, 0, Doy, 0, 0, 0, Doz;

  // position integral in desired frame
  Ip_d_target << Ipx, 0, 0, 0, Ipy, 0, 0, 0, Ipz;

  // orientation integral in desired frame
  Io_d_target << Iox, 0, 0, 0, Ioy, 0, 0, 0, Ioz;

  // nullspace Gains
  null_K_d_target = Kpn * null_K_d_target.setIdentity();
}

/**
 * \brief Calculate the joint derivative between its limits.
 */
double RobotArmController::calcJointDerivative(const double q_i, const double min_joint_limit_i,
                                               const double max_joint_limit_i)
{
  double average_joint = (max_joint_limit_i + min_joint_limit_i) / 2.0;
  return -(((q_i - average_joint) / pow((max_joint_limit_i - min_joint_limit_i), 2)));
}

/**
 * \brief Calculates distance from mechanical joint limits null space objective function.
 */
Eigen::Matrix<double, 7, 1> RobotArmController::distanceFromMechanicalJointLimits(
    const Eigen::Matrix<double, 7, 1> q, const Eigen::Matrix<double, 7, 1> min_joint_limits,
    const Eigen::Matrix<double, 7, 1> max_joint_limits)
{
  Eigen::Matrix<double, 7, 1> distance_mechanical_joint_limits;
  for (int i = 0; i < q.rows(); i++)
    distance_mechanical_joint_limits(i) = calcJointDerivative(q(i), min_joint_limits(i), max_joint_limits(i));

  return distance_mechanical_joint_limits;
}

/**
 * \brief Calculates null space objective function.
 *
 * Uses the distance from mechanical joint limits as the objective function for null space.
 */
void RobotArmController::nullSpaceObjectiveFunction(Eigen::Matrix<double, 7, 1> &objective)
{
  objective = distanceFromMechanicalJointLimits(q_, min_joint_limits_, max_joint_limits_);
}

/**
 * \fn void filterGains(void)
 * \brief Filter dynamic reconfigure impedance gains.
 */
void RobotArmController::filterGains(void)
{
  Kp_d = filter_param_ * Kp_d_target + (1.0 - filter_param_) * Kp_d;
  Dp_d = filter_param_ * Dp_d_target + (1.0 - filter_param_) * Dp_d;
  Ko_d = filter_param_ * Ko_d_target + (1.0 - filter_param_) * Ko_d;
  Do_d = filter_param_ * Do_d_target + (1.0 - filter_param_) * Do_d;
  Ip_d = filter_param_ * Ip_d_target + (1.0 - filter_param_) * Ip_d;
  Io_d = filter_param_ * Io_d_target + (1.0 - filter_param_) * Io_d;
  null_K_d = filter_param_ * null_K_d_target + (1.0 - filter_param_) * null_K_d;
}

/**
 * \fn void setControllerGains(void)
 * \brief Filters the gains, changes to base frame and updates impedance matrices.
 */
void RobotArmController::setControllerGains(void)
{
  // Filter the gains so that the response is not abrupt!
  filterGains();

  // Change from end-effector frame to base frame
  Eigen::Matrix3d Kp(R0ee_d_ * Kp_d * R0ee_d_.transpose());  // cartesian position stiffness
  Eigen::Matrix3d Dp(R0ee_d_ * Dp_d * R0ee_d_.transpose());  // cartesian position damping
  Eigen::Matrix3d Ip(R0ee_d_ * Ip_d * R0ee_d_.transpose());  // cartesian position integral
  Eigen::Matrix3d Ko(R0ee_d_ * Ko_d * R0ee_d_.transpose());  // cartesian orientation stiffness
  Eigen::Matrix3d Do(R0ee_d_ * Do_d * R0ee_d_.transpose());  // cartesian orientation damping
  Eigen::Matrix3d Io(R0ee_d_ * Io_d * R0ee_d_.transpose());  // cartesian orientation integral

  cart_K.setIdentity();
  cart_K.topLeftCorner(3, 3) << Kp;
  cart_K.bottomRightCorner(3, 3) << Ko;

  cart_D.setIdentity();
  cart_D.topLeftCorner(3, 3) << Dp;
  cart_D.bottomRightCorner(3, 3) << Do;

  cart_I.setIdentity();
  cart_I.topLeftCorner(3, 3) << Ip;
  cart_I.bottomRightCorner(3, 3) << Io;

  null_K.setIdentity();
  null_K = null_K_d;
}

/**
 * \brief Calculates the accumulated end-effector error (position & orientation), over
 * time for the impedance integral block.
 *
 * The position error is computed via a simple addition of the current error to the previous errors.
 * The orientation error is not as simple.
 */
void RobotArmController::calcIntegralError(void)
{
  error_accum_.head(3) = error_prev_.head(3) + error_.head(3);  // position

  if (error_.tail(3).transpose() * error_.tail(3) < 0.00001)  // orientation
    error_accum_.tail(3) = error_.tail(3);
  else
    error_accum_.tail(3) = (error_prev_.tail(3).transpose() * (error_.tail(3) / error_.tail(3).norm()) *
                            (error_.tail(3) / error_.tail(3).norm())) +
                           error_.tail(3);
}

/**
 * \brief Saturates the integral error to prevent overdriving the joint actuators.
 *
 * If the error goes above positive clamping limit or below negative clamping limit
 * it should use the respective limit values.
 */
void RobotArmController::saturateIntegralError(void)
{
  for (size_t i = 0; i < 3; i++)  // position error
  {
    if (error_accum_[i] > i_clamp_p_)
      error_accum_[i] = i_clamp_p_;
    else if (error_accum_[i] < -i_clamp_p_)
      error_accum_[i] = -i_clamp_p_;
  }
  for (size_t i = 3; i < 6; i++)  // orientation error
  {
    if (error_accum_[i] > i_clamp_o_)
      error_accum_[i] = i_clamp_o_;
    else if (error_accum_[i] < -i_clamp_o_)
      error_accum_[i] = -i_clamp_o_;
  }
}

}  // namespace smalldrop_robot_arm