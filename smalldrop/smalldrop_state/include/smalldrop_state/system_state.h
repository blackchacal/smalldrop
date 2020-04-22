// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file system_state.h
 * \brief Declares a class that models the smalldrop system state.
 */

#ifndef _SMALLDROP_SYSTEM_STATE_H
#define _SMALLDROP_SYSTEM_STATE_H

#include <ros/ros.h>

// ROS messages
#include <smalldrop_msgs/JointPositions.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <controller_manager_msgs/SwitchController.h>

namespace smalldrop
{
namespace smalldrop_state
{
class SystemState
{
private:
  /**
   * Class members
   *****************************************************************************************/

  // State variables
  std::string system_state_;                            /** \var Stores the system working state gathered from a ROS topic. */
  sensor_msgs::Joy remote_ctrl_state_;                  /** \var Stores the state of space mouse controller gathered from a ROS topic. */
  geometry_msgs::Pose robot_arm_pose_;                  /** \var Stores the robot arm pose. */
  smalldrop_msgs::JointPositions robot_arm_joint_pos_;  /** \var Stores the robot arm joint positions. */

  // ROS Topics
  std::string system_state_topic_;                /** \var ROS topic where system working state is published. */
  std::string remote_ctrl_state_topic_;           /** \var ROS topic where space mouse state is published. */
  std::string robot_arm_current_pose_topic_;      /** \var ROS topic where robot arm current pose is published. */
  std::string robot_arm_desired_pose_topic_;      /** \var ROS topic where robot arm desired pose is published. */
  std::string robot_arm_current_joint_pos_topic_; /** \var ROS topic where robot arm current joint positions are published. */
  std::string robot_arm_desired_joint_pos_topic_; /** \var ROS topic where robot arm desired joint positions are published. */
  std::string rviz_segmentation_points_topic_;    /** \var ROS topic where wound segmentation points are published. */
  
  std::string controller_manager_switch_controller_srv_topic_; /** \var ROS service topic for controller manager switch controller service. */

  // Node handle
  ros::NodeHandle nh_; /** \var ROS node handle to access topics system. */

  // Subscribers
  ros::Subscriber system_state_sub_; /** \var ROS topic subscriber instance to subscribe to system working state topic. */
  ros::Subscriber remote_ctrl_state_sub_; /** \var ROS topic subscriber instance to subscribe to remote controller state topic. */
  ros::Subscriber robot_arm_state_sub_; /** \var ROS topic subscriber instance to subscribe to robot arm state topic. */
  ros::Subscriber robot_arm_joint_pos_sub_; /** \var ROS topic subscriber instance to subscribe to robot arm joint positions topic. */

  // Publishers
  ros::Publisher robot_arm_desired_pose_pub_;      /** \var ROS publisher for the robot arm desired pose. */
  ros::Publisher robot_arm_desired_joint_pos_pub_; /** \var ROS publisher for the robot arm desired joint positions. */
  ros::Publisher rviz_trajectory_markers_pub_;     /** \var ROS publisher for rviz trajectory markers. */
  ros::Publisher rviz_segmentation_points_pub_;    /** \var ROS publisher for rviz segmentation points markers. */

  // Service Clients
  ros::ServiceClient controller_manager_switch_controller_srv_; /** \var ROS service client for service calls to controller_manager. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn void subscribeTopics()
   * \brief Subscribe to all the topics related to the smalldrop system state.
   */
  void subscribeTopics();

  /**
   * \fn void systemStateCallback(const std_msgs::String::ConstPtr &msg)
   * \brief Callback to gather the data published on the system working state topic.
   */
  void systemStateCallback(const std_msgs::String::ConstPtr &msg);

  /**
   * \fn void robotArmStateCallback(const geometry_msgs::Pose::ConstPtr &msg)
   * \brief Callback to gather the data published on the robot arm state topic.
   */
  void robotArmStateCallback(const geometry_msgs::Pose::ConstPtr &msg);

  /**
   * \fn void robotArmJointPositionsCallback(const smalldrop_msgs::JointPositions::ConstPtr &msg)
   * \brief Callback to gather the data published on the robot arm joint positions topic.
   */
  void robotArmJointPositionsCallback(const smalldrop_msgs::JointPositions::ConstPtr &msg);

  /**
   * \fn void remoteCtrlStateCallback(const sensor_msgs::Joy::ConstPtr &msg)
   * \brief Callback to gather the data published on the remote controller state topic.
   */
  void remoteCtrlStateCallback(const sensor_msgs::Joy::ConstPtr &msg);

public:
  // Constructor
  SystemState();

  /**
   * \fn std_msgs::String getSystemState() const
   * \brief Returns the system working state.
   */
  std::string getSystemState() const;

  /**
   * \fn geometry_msgs::Pose getRobotArmPose() const
   * \brief Returns the robot arm current pose.
   */
  geometry_msgs::Pose getRobotArmPose() const;

  /**
   * \fn smalldrop_msgs::JointPositions getRobotArmJointPositions() const
   * \brief Returns the robot arm current joint positions.
   */
  smalldrop_msgs::JointPositions getRobotArmJointPositions() const;

  /**
   * \fn sensor_msgs::Joy getRemoteCtrlState() const
   * \brief Returns the remote controller current state.
   */
  sensor_msgs::Joy getRemoteCtrlState() const;

  /**
   * \fn ros::Publisher getRobotDesiredPosePublisher() const
   * \brief Returns the publisher for robot arm desired pose.
   */
  ros::Publisher getRobotDesiredPosePublisher() const;

  /**
   * \fn ros::Publisher getRobotDesiredJointPositionsPublisher() const
   * \brief Returns the publisher for robot arm desired joint positions.
   */
  ros::Publisher getRobotDesiredJointPositionsPublisher() const;

  /**
   * \fn ros::Publisher getTrajectoryMarkersPublisher() const
   * \brief Returns the publisher for robot arm trajectory markers.
   */
  ros::Publisher getTrajectoryMarkersPublisher() const;

  /**
   * \fn ros::Publisher getSegmentationPointsPublisher() const
   * \brief Returns the publisher for segmentation points markers.
   */
  ros::Publisher getSegmentationPointsPublisher() const;

  /**
   * \fn ros::ServiceClient getControllerManagerSwitchControllerService() const
   * \brief Returns the service client associated with the controller manager switch controller service.
   */
  ros::ServiceClient getControllerManagerSwitchControllerService() const;
};

}  // namespace smalldrop_state

}  // namespace smalldrop

#endif  // _SMALLDROP_SYSTEM_STATE_H