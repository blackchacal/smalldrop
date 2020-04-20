// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file move_to_joint_config.cpp
 * \brief ROS node to move the robot arm to a chosen joint configuration.
 */

#include <ros/ros.h>
#include <smalldrop_toolpath/joint_trajectory_planner.h>

// ROS messages
#include <smalldrop_msgs/JointPositions.h>

using namespace smalldrop::smalldrop_toolpath;

/**
 * Global variables
 *********************************************************************************************/

jpos_t joints_i = {0, 0, 0, 0, 0, 0, 0};
jpos_t joints_f = {0, 0, 0, 0, 0, 0, 0};
std::string joints_str;
bool has_pose = false;
int freq = 100; // 100 Hz
double ttime = 10; // Trajectory duration in seconds

/**
 * Function prototypes
 *********************************************************************************************/

bool processCmdArgs(int argc, char **argv);
void currentJointConfigCallback(smalldrop_msgs::JointPositions::ConstPtr msg);

/**
 * Main
 *********************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_to_joint_config");
  ros::NodeHandle nh;

  ros::Publisher joints_pub = nh.advertise<smalldrop_msgs::JointPositions>("/smalldrop/robot_arm/desired_joint_positions", 10);
  ros::Subscriber sub = nh.subscribe("/smalldrop/robot_arm/current_joint_positions", 10, currentJointConfigCallback);

  // If no joint configurations are set, keep the current configuration
  joints_f = joints_i;

  if (!processCmdArgs(argc, argv))
    return 0;

  // Process current pose callback
  ros::Rate r(10);
  while (ros::ok() && !has_pose)
  {
    ros::spinOnce();
    r.sleep();
  }

  // Plan joint trajectory between joints_i and joints_f
  JointTrajectoryPlanner pl(ttime, freq, PLAN_MODE::POLY3);  
  std::vector<jpos_t> traj = pl.plan(joints_i, joints_f);

  ros::Rate r2(freq);
  for (size_t i = 0; i < traj.size(); i++)
  {
    smalldrop_msgs::JointPositions msg;
    for (size_t j = 0; j < traj[i].size(); j++)
      msg.positions.push_back(traj[i][j]);

    // Send trajectory to robot
    joints_pub.publish(msg);
    ros::spinOnce();
    r2.sleep();
  }
  
  return 0;
}

/**
 * General functions & callbacks
 *********************************************************************************************/

bool processCmdArgs(int argc, char **argv)
{
  // Process command-line arguments
  int opt;
  const char* const short_opts = ":hf:j:t:";
  while ((opt = getopt(argc, argv, short_opts)) != -1)
  {
    switch (opt)
    {
      case 'f':
        freq = std::stoi(optarg);
        break;
      case 'j':
        {
          joints_str = optarg;
          // Define new joint configuration (to where the robot should move)
          std::stringstream ss(joints_str);
          int i = 0;
          for (double val; ss >> val;) 
          {
            joints_f[i++] = val;    
            if (ss.peek() == ',')
              ss.ignore();
          }
        }
        break;
      case 't':
        ttime = std::stod(optarg);
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "move_to_joint_config -h -j <joint_list> -t <trajectory_time> -f <frequency>" << std::endl; 
        return false;
    }
  }
  return true;
}

void currentJointConfigCallback(smalldrop_msgs::JointPositions::ConstPtr msg)
{
  for (size_t i = 0; i < msg->positions.size(); i++)
    joints_i[i] = msg->positions[i];
  
  has_pose = true;
}