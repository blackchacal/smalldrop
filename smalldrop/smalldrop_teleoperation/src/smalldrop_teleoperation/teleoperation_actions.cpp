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

int segmentation_marker_id = 0; /** \var Wound segmentation markers id. */

/**
 * \copybrief changeMode(smalldrop_state::SystemState* system_state)
 */
bool changeMode(smalldrop_state::SystemState* system_state)
{
  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";
  ros::Publisher rviz_segmentation_points_pub = system_state->getSegmentationPointsPublisher();

  // Open file to append point data
  std::fstream fh;

  fh.open(filepath, std::fstream::out);
  fh << "px py pz ox oy oz ow" << "\n";
  fh.close();

  segmentation_marker_id = 0;

  // Delete previous markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "panda_link0";
  marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.action = visualization_msgs::Marker::DELETEALL;
  rviz_segmentation_points_pub.publish(marker);

  return true;
}

/**
 * \copybrief moveRobotArm(smalldrop_state::SystemState* system_state)
 */
bool moveRobotArm(smalldrop_state::SystemState* system_state)
{
  ROS_INFO_NAMED(LOG_TAG, "%s: Moving Robot Arm...", LOG_TAG.c_str());

  double sensitivity_factor = 0.01;

  sensor_msgs::Joy remote_ctrl_state = system_state->getRemoteCtrlState();
  geometry_msgs::Pose robot_arm_pose = system_state->getRobotArmPose();
  ros::Publisher robot_arm_desired_pose_pub = system_state->getRobotDesiredPosePublisher();

  if (remote_ctrl_state.axes.size() >= 6)
  {
    std::cout << remote_ctrl_state.axes.size() << std::endl;
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
  return false;
}

/**
 * \copybrief publishSegmentationPoint(smalldrop_state::SystemState* system_state)
 */
bool publishSegmentationPoint(smalldrop_state::SystemState* system_state)
{
  ROS_INFO_NAMED(LOG_TAG, "%s: Publish Segmentation Point...", LOG_TAG.c_str());

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";
  geometry_msgs::Pose robot_arm_pose = system_state->getRobotArmPose();
  ros::Publisher rviz_segmentation_points_pub = system_state->getSegmentationPointsPublisher();

  // Open file to append point data
  std::fstream fh;

  fh.open(filepath, std::fstream::app);
  if (fh.is_open())
  {
    fh << robot_arm_pose.position.x << " " << robot_arm_pose.position.y << " " << robot_arm_pose.position.z \
    << " " << robot_arm_pose.orientation.x << " " << robot_arm_pose.orientation.y << " " << robot_arm_pose.orientation.z << " " << robot_arm_pose.orientation.w << "\n";
    ROS_INFO_NAMED(LOG_TAG, "%s: Stored pose: p(%.4f, %.4f, %.4f) o(%.4f, %.4f, %.4f, %.4f).", LOG_TAG.c_str(), robot_arm_pose.position.x, robot_arm_pose.position.y, robot_arm_pose.position.z, \
    robot_arm_pose.orientation.x, robot_arm_pose.orientation.y, robot_arm_pose.orientation.z, robot_arm_pose.orientation.w);
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.ns = "/smalldrop/teleoperation/segmentation_points";
    marker.id = segmentation_marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = robot_arm_pose;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    rviz_segmentation_points_pub.publish(marker);

    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "panda_link0";
    marker_text.header.stamp = ros::Time();
    marker_text.ns = "/smalldrop/teleoperation/segmentation_points_text";
    marker_text.id = segmentation_marker_id++;
    marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_text.action = visualization_msgs::Marker::ADD;
    marker_text.pose = robot_arm_pose;
    marker_text.pose.position.z = robot_arm_pose.position.z + 0.03;
    marker_text.scale.z = 0.03;
    marker_text.color.r = 1.0;
    marker_text.color.g = 1.0;
    marker_text.color.b = 0;
    marker_text.color.a = 1.0;
    std::ostringstream txt;
    txt.precision(3);
    txt << "(" << robot_arm_pose.position.x << "," << robot_arm_pose.position.y << "," << robot_arm_pose.position.z << ")";
    marker_text.text = txt.str();
    rviz_segmentation_points_pub.publish(marker_text);

    fh.close(); // Close file
  }
  else
  {
    ROS_ERROR("The files were not properly opened!");
  }

  return true;
}

}  // namespace teleop_actions

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop