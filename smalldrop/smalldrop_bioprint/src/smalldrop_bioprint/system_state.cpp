// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file system_state.cpp
 * \brief Defines a class that models the smalldrop system state.
 */

#include <smalldrop_bioprint/system_state.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{

/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

SystemState::SystemState()
{
  remote_ctrl_state_topic_ = "/spacenav/joy";
  robot_arm_current_pose_topic_ = "/smalldrop/robot_arm/current_pose";
  robot_arm_desired_pose_topic_ = "/smalldrop/robot_arm/desired_pose";
  rviz_segmentation_points_topic_ = "/smalldrop/teleoperation/segmentation_points";

  remote_ctrl_state_.axes.reserve(6);
  remote_ctrl_state_.buttons.reserve(2);
  remote_ctrl_state_.axes[0] = 0;
  remote_ctrl_state_.axes[1] = 0;
  remote_ctrl_state_.axes[2] = 0;
  remote_ctrl_state_.axes[3] = 0;
  remote_ctrl_state_.axes[4] = 0;
  remote_ctrl_state_.axes[5] = 0;
  remote_ctrl_state_.buttons[0] = 0;
  remote_ctrl_state_.buttons[1] = 0;

  subscribeTopics();
}

/**
 * \copybrief SystemState::getRobotArmPose() const
 */
geometry_msgs::Pose SystemState::getRobotArmPose() const
{
  return robot_arm_pose_;
}

/**
 * \copybrief SystemState::getRemoteCtrlState() const
 */
sensor_msgs::Joy SystemState::getRemoteCtrlState() const
{
  return remote_ctrl_state_;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief SystemState::subscribeTopics()
 */
void SystemState::subscribeTopics()
{
  robot_arm_state_sub_ = nh_.subscribe<geometry_msgs::Pose>(robot_arm_current_pose_topic_, 10, &SystemState::robotArmStateCallback, this);
  remote_ctrl_state_sub_ = nh_.subscribe<sensor_msgs::Joy>(remote_ctrl_state_topic_, 10, &SystemState::remoteCtrlStateCallback, this);

  robot_arm_desired_pose_pub_ = nh_.advertise<geometry_msgs::Pose>(robot_arm_desired_pose_topic_, 10);
  rviz_segmentation_points_pub_ = nh_.advertise<visualization_msgs::Marker>(rviz_segmentation_points_topic_, 10);
}

/**
 * \copybrief SystemState::robotArmStateCallback()
 */
void SystemState::robotArmStateCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
  robot_arm_pose_.position = msg->position;
  robot_arm_pose_.orientation = msg->orientation;
}

/**
 * \copybrief SystemState::remoteCtrlStateCallback()
 */
void SystemState::remoteCtrlStateCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  remote_ctrl_state_.header = msg->header;
  remote_ctrl_state_.axes = msg->axes;
  remote_ctrl_state_.buttons = msg->buttons;
}

/**
 * \copybrief SystemState::getRobotDesiredPosePublisher() const
 */
ros::Publisher SystemState::getRobotDesiredPosePublisher() const
{
  return robot_arm_desired_pose_pub_;
}

/**
 * \copybrief SystemState::getTrajectoryMarkersPublisher() const
 */
ros::Publisher SystemState::getTrajectoryMarkersPublisher() const
{
  return rviz_trajectory_markers_pub_;
}

/**
 * \copybrief SystemState::getSegmentationPointsPublisher() const
 */
ros::Publisher SystemState::getSegmentationPointsPublisher() const
{
  return rviz_segmentation_points_pub_;
}

}  // namespace smalldrop_bioprint

}  // namespace smalldrop