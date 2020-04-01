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
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <smalldrop_bioprint/bioprinter.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{
class SystemState
{
private:
  /**
   * Class members
   *****************************************************************************************/

  // State variables
  std::string system_state_;           /** \var Stores the system working state gathered from a ROS topic. */
  sensor_msgs::Joy remote_ctrl_state_; /** \var Stores the state of space mouse controller gathered from a ROS topic. */
  geometry_msgs::Pose robot_arm_pose_; /** \var Stores the robot arm pose. */

  // ROS Topics
  std::string system_state_topic_;              /** \var ROS topic where system working state is published. */
  std::string remote_ctrl_state_topic_;         /** \var ROS topic where space mouse state is published. */
  std::string robot_arm_current_pose_topic_;    /** \var ROS topic where space mouse current pose is published. */
  std::string robot_arm_desired_pose_topic_;    /** \var ROS topic where space mouse desired pose is published. */
  std::string rviz_segmentation_points_topic_;  /** \var ROS topic where wound segmentation points are published. */

  // Node handle
  ros::NodeHandle nh_;                          /** \var ROS node handle to access topics system. */

  // Subscribers
  ros::Subscriber system_state_sub_;            /** \var ROS topic subscriber instance to subscribe to system working state topic. */
  ros::Subscriber remote_ctrl_state_sub_;       /** \var ROS topic subscriber instance to subscribe to remote controller state topic. */
  ros::Subscriber robot_arm_state_sub_;         /** \var ROS topic subscriber instance to subscribe to robot arm state topic. */

  // Publishers
  ros::Publisher robot_arm_desired_pose_pub_;   /** \var ROS publisher for the robot arm desired pose. */
  ros::Publisher rviz_trajectory_markers_pub_;  /** \var ROS publisher for rviz trajectory markers. */
  ros::Publisher rviz_segmentation_points_pub_; /** \var ROS publisher for rviz segmentation points markers. */

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
   * \fn ros::Publisher getTrajectoryMarkersPublisher() const
   * \brief Returns the publisher for robot arm trajectory markers.
   */
  ros::Publisher getTrajectoryMarkersPublisher() const;

  /**
   * \fn ros::Publisher getSegmentationPointsPublisher() const
   * \brief Returns the publisher for segmentation points markers.
   */
  ros::Publisher getSegmentationPointsPublisher() const;
};

}  // namespace smalldrop_bioprint

}  // namespace smalldrop

#endif // _SMALLDROP_SYSTEM_STATE_H