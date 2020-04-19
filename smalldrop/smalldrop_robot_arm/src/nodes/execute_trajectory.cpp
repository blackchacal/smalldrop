// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file execute_trajectory.cpp
 * \brief ROS node to send trajectory poses to the robot arm.
 */

#include <ros/ros.h>
#include <getopt.h>

#include <smalldrop_toolpath/trajectory.h>
#include <smalldrop_toolpath/trajectory_planner.h>
#include <smalldrop_toolpath/line.h>
#include <smalldrop_toolpath/circle.h>
#include <smalldrop_toolpath/circular_spiral.h>

// ROS messages
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// Libraries
#include <Eigen/Dense>

using namespace smalldrop::smalldrop_toolpath;

/**
 * Global variables
 *********************************************************************************************/

pose_t initial_pose;
pose_t final_pose;
pose_t center;
bool has_pose = false;
bool show_tf = false;
bool show_markers = false;
bool show_time = false;
bool show_size = false;
bool send_robot = false;
std::string path_type = "";
double ttime = 10; // Trajectory duration in seconds
double loops = 5; // circular trajectory number of loops
int freq = 100; // 100 Hz
double radius = 0.1; // circular path radius
double eradius = 0.1; // circular spiral path external radius
double iradius = 0.0; // circular spiral path internal radius
std::string plane = "xy";

/**
 * Function prototypes
 *********************************************************************************************/

void currentPoseCallback(geometry_msgs::Pose::ConstPtr msg);
bool processCmdArgs(int argc, char **argv);

/**
 * Main
 *********************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_trajectory_node");
  ros::NodeHandle nh;

  static tf::TransformBroadcaster br;
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/smalldrop/robot_arm/desired_pose", 10);
  ros::Publisher pub_markers = nh.advertise<visualization_msgs::Marker>("/smalldrop/robot_arm/trajectory_markers", 10);
  ros::Subscriber sub = nh.subscribe("/smalldrop/robot_arm/current_pose", 10, currentPoseCallback);

  // Process current pose callback
  ros::Rate r(10);
  while (ros::ok() && !has_pose)
  {
    ros::spinOnce();
    r.sleep();
  }

  // We have the initial pose. Set final and center poses
  final_pose = initial_pose;
  center = initial_pose;

  // process command line arguments
  // change the final_pose or center pose depending on the arguments
  if (!processCmdArgs(argc, argv)) 
  {
    return 0;
  }

  ros::Rate rate(freq);
  
  ros::Time ts, te;
  double time_elapsed;

  poses_t traj;
  PATH_PLANE path_plane = PATH_PLANE::XY;

  
  if (plane.compare("xz") == 0)
    path_plane = PATH_PLANE::XZ;
  if (plane.compare("yz") == 0)
    path_plane = PATH_PLANE::YZ;
  
  TrajectoryPlanner pl(ttime, freq, PLAN_MODE::LSPB);
  if (path_type.compare("line") == 0)
  {
    Line line(initial_pose, final_pose);
    Trajectory t(pl.plan(line));
    traj = t.poses();
  }
  else if (path_type.compare("circle") == 0)
  {
    pose_t start_pose; 
    switch (path_plane)
      {
      case PATH_PLANE::XZ:
        start_pose.position.x = radius + center.position.x;
        start_pose.position.y = center.position.y;
        start_pose.position.z = center.position.z;
        break;
      case PATH_PLANE::YZ:
        start_pose.position.x = center.position.x;
        start_pose.position.y = radius + center.position.y;
        start_pose.position.z = center.position.z;;
        break;
      default:
        start_pose.position.x = radius + center.position.x;
        start_pose.position.y = center.position.y;
        start_pose.position.z = center.position.z;
        break;
      }
    start_pose.orientation.x = initial_pose.orientation.x;
    start_pose.orientation.y = initial_pose.orientation.y;
    start_pose.orientation.z = initial_pose.orientation.z;
    start_pose.orientation.w = initial_pose.orientation.w;

    Line line(initial_pose, start_pose);
    Circle circle(start_pose, center, radius, 20, path_plane);
    std::vector<Path> paths = {line, circle};
    Trajectory t(pl.plan(paths));
    traj = t.poses();
  }
  else if (path_type.compare("circular_spiral") == 0)
  {
    CircularSpiral circular_spiral(initial_pose, eradius, iradius, loops, 20, path_plane);
    Trajectory t(pl.plan(circular_spiral));
    traj = t.poses();
  }
  else
  {
    ROS_WARN("No valid path chosen!");
    return 0;
  }
  
  if (show_size)
  {
    std::cout << "Trajectory size: " << traj.size() << std::endl;
  }
  
  if (show_time) {
    ts = ros::Time::now();
  }
  for (size_t i = 0; i < traj.size(); i++)
  {
    pose_t p = traj[i];
    if (show_markers && i < traj.size()-1)
    {
      std::vector<geometry_msgs::Point> points;
      geometry_msgs::Point p1;
      geometry_msgs::Point p2;
      p1.x = traj[i].position.x;
      p1.y = traj[i].position.y;
      p1.z = traj[i].position.z;
      p2.x = traj[i+1].position.x;
      p2.y = traj[i+1].position.y;
      p2.z = traj[i+1].position.z;
      points.push_back(p1);
      points.push_back(p2);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "panda_link0";
      marker.header.stamp = ros::Time();
      marker.ns = "panda_3dbioprint_simulation";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.points = points;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.003;
      marker.color.r = 1.0;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1.0;
      pub_markers.publish(marker);
    }

    // Broadcast tf of trajectory points
    if (show_tf) 
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(p.position.x, p.position.y, p.position.z) );
      tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", "trajectory"));
    }

    // Send trajectory to robot
    if (send_robot) 
    {
      pub.publish(p);
    }

    ros::spinOnce();
    rate.sleep();
  }

  if (show_time) 
  {
    te = ros::Time::now();
    time_elapsed = (double)(te.sec - ts.sec) + (double)(te.nsec - ts.nsec) / 1000000000;
    std::cout << "Time elapsed: " << time_elapsed << std::endl;
  }
  
  return 0;
}

/**
 * General functions & callbacks
 *********************************************************************************************/

void currentPoseCallback(geometry_msgs::PoseConstPtr msg)
{
  initial_pose.position.x = msg->position.x;
  initial_pose.position.y = msg->position.y;
  initial_pose.position.z = msg->position.z;
  initial_pose.orientation.x = msg->orientation.x;
  initial_pose.orientation.y = msg->orientation.y;
  initial_pose.orientation.z = msg->orientation.z;
  initial_pose.orientation.w = msg->orientation.w;

  has_pose = true;
}

bool processCmdArgs(int argc, char **argv)
{
  // Process command-line arguments
  int opt;
  const char* const short_opts = ":smrtfp:x:y:z:q:u:v:w:T:L:F:R:E:I:P:";
  const option long_opts[] = {
    {"rx", required_argument, nullptr, 'q'},
    {"ry", required_argument, nullptr, 'u'},
    {"rz", required_argument, nullptr, 'v'},
    {"rw", required_argument, nullptr, 'w'},
    {"time", required_argument, nullptr, 'T'},
    {"freq", required_argument, nullptr, 'F'},
    {"radius", required_argument, nullptr, 'R'},
    {"eradius", required_argument, nullptr, 'E'},
    {"iradius", required_argument, nullptr, 'I'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, no_argument, nullptr, 0}
  };

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1)
  {
    switch (opt)
    {
      case 's':
        show_size = true;
        break;
      case 'm':
        show_markers = true;
        break;
      case 'r':
        send_robot = true;
        break;
      case 't':
        show_time = true;
        break;
      case 'f':
        show_tf = true;
        break;
      case 'p':
        path_type = optarg;
        break;
      case 'x':
        if (path_type == "line")
          final_pose.position.x = std::stod(optarg);
        else
          center.position.x = std::stod(optarg);
        break;
      case 'y':
        if (path_type == "line")
          final_pose.position.y = std::stod(optarg);
        else
          center.position.y = std::stod(optarg);
        break;
      case 'z':
        if (path_type == "line")
          final_pose.position.z = std::stod(optarg);
        else
          center.position.z = std::stod(optarg);
        break;
      case 'q':
        if (path_type == "line")
          final_pose.orientation.x = std::stod(optarg);
        else
          center.orientation.x = std::stod(optarg);
        break;
      case 'u':
        if (path_type == "line")
          final_pose.orientation.y = std::stod(optarg);
        else
          center.orientation.y = std::stod(optarg);
        break;
      case 'v':
        if (path_type == "line")
          final_pose.orientation.z = std::stod(optarg);
        else
          center.orientation.z = std::stod(optarg);
        break;
      case 'w':
        if (path_type == "line")
          final_pose.orientation.w = std::stod(optarg);
        else
          center.orientation.w = std::stod(optarg);
        break;
      case 'L':
        loops = std::stoi(optarg);
        break;
      case 'T':
        ttime = std::stod(optarg);
        break;
      case 'F':
        freq = std::stoi(optarg);
        break;
      case 'R':
        radius = std::stod(optarg);
        break;
      case 'E':
        eradius = std::stod(optarg);
        break;
      case 'I':
        iradius = std::stod(optarg);
        break;
      case 'P':
        plane = static_cast<std::string>(optarg);
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "execute_trajectory -smrtf -p <path_type> -P <trajectory_plane> -T <trajectory_time> -L <number_loops> \
        --freq <frequency> --radius <radius> --eradius <external_radius> --iradius <internal_radius> \
        -x <pos_x> -y <pos_y> -z <pos_z> --rx <orient_x> --ry <orient_y> --rz <orient_z> --rw <orient_w>" << std::endl; 
        std::cout << "s: show trajectory size" << std::endl; 
        std::cout << "m: show trajectory markers" << std::endl; 
        std::cout << "r: send trajectory to robot" << std::endl; 
        std::cout << "t: show trajectory planning time" << std::endl; 
        std::cout << "f: show trajectory tf" << std::endl;
        std::cout << "p: path type: line, circle, circular_spiral" << std::endl;
        std::cout << "P: Trajectory plane: xy, xz, yz" << std::endl;
        return false;
        break;
    }
  }
  return true;
}