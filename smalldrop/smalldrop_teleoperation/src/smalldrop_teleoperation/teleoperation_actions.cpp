// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file teleoperation_actions.cpp
 * \brief Defines set of functions (actions) for robot arm teleoperation.
 */

#include <smalldrop_teleoperation/teleoperation_actions.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
namespace teleop_actions
{

const std::string LOG_TAG = "smalldrop_teleoperation";

bool moveRobotArm(smalldrop_bioprint::SystemState* system_state)
{
  ROS_INFO_NAMED(LOG_TAG, "%s: Moving Robot Arm...", LOG_TAG.c_str());

  double sensitivity_factor = 0.01;

  sensor_msgs::Joy remote_ctrl_state = system_state->getRemoteCtrlState();
  geometry_msgs::Pose robot_arm_pose = system_state->getRobotArmPose();
  ros::Publisher robot_arm_desired_pose_pub = system_state->getRobotDesiredPosePublisher();

  // Remote controller joy data
  float posx = remote_ctrl_state.axes[0];
  float posy = remote_ctrl_state.axes[1];
  float posz = remote_ctrl_state.axes[2];
  float orientx = remote_ctrl_state.axes[3];
  float orienty = remote_ctrl_state.axes[4];
  float orientz = remote_ctrl_state.axes[5];

  float orient_sensitivity = sensitivity_factor * 10;

  geometry_msgs::Pose new_pose;
  new_pose.position.x = robot_arm_pose.position.x + sensitivity_factor * (-posx);
  new_pose.position.y = robot_arm_pose.position.y + sensitivity_factor * (-posy);
  new_pose.position.z = robot_arm_pose.position.z + sensitivity_factor * posz;

  Eigen::Quaterniond current_orientation(robot_arm_pose.orientation.w, 
                                        robot_arm_pose.orientation.x, 
                                        robot_arm_pose.orientation.y, 
                                        robot_arm_pose.orientation.z);

  Eigen::Matrix3d Rc = current_orientation.matrix();  // Rotation matrix for current pose
  Eigen::Matrix3d Rs;                                 // Rotation matrix for spacenav
  Eigen::Matrix3d Rsx;                                // Rotation matrix for spacenav x-axis rotation
  Eigen::Matrix3d Rsy;                                // Rotation matrix for spacenav y-axis rotation
  Eigen::Matrix3d Rsz;                                // Rotation matrix for spacenav z-axis rotation

  if (orientx >= 0 && orientx < 0.001 && orienty >= 0 && orienty < 0.001 && orientz >= 0 && orientz < 0.001)
  {
    Rs << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  }
  else
  {
    Rsx << 1, 0, 0, 0, cos(orient_sensitivity * orientx), -sin(orient_sensitivity * orientx), 0,
        sin(orient_sensitivity * orientx), cos(orient_sensitivity * orientx);

    Rsy << cos(-orient_sensitivity * orienty), 0, sin(-orient_sensitivity * orienty), 0, 1, 0,
        -sin(-orient_sensitivity * orienty), 0, cos(-orient_sensitivity * orienty);

    Rsz << cos(-orient_sensitivity * orientz), -sin(-orient_sensitivity * orientz), 0,
        sin(-orient_sensitivity * orientz), cos(-orient_sensitivity * orientz), 0, 0, 0, 1;

    Rs = Rsx * Rsy * Rsz;
  }

  Eigen::Matrix3d Rf = Rc * Rs;  // In relation to end-effector
  // Eigen::Matrix3d Rf = Rs * Rc; // In relation to base

  Eigen::Quaterniond new_orientation(Rf);

  new_pose.orientation.x = new_orientation.x();
  new_pose.orientation.y = new_orientation.y();
  new_pose.orientation.z = new_orientation.z();
  new_pose.orientation.w = new_orientation.w();

  if (posx || posy || posz || orientx || orienty || orientz)
    robot_arm_desired_pose_pub.publish(new_pose);

  return true;
}

}  // namespace teleop_actions

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop