// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file execute_trajectory.cpp
 * \brief ROS node to send trajectory poses to the robot arm.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <getopt.h>

#include <smalldrop_vision/camera_d415.h>
#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>
#include <smalldrop_segmentation/wound_segmentation_comanip_convex_hull.h>

#include <smalldrop_toolpath/trajectory.h>
#include <smalldrop_toolpath/trajectory_planner.h>
#include <smalldrop_toolpath/line.h>
#include <smalldrop_toolpath/circle.h>
#include <smalldrop_toolpath/circular_spiral.h>
#include <smalldrop_toolpath/zigzag.h>
#include <smalldrop_toolpath/parallel_lines.h>
#include <smalldrop_toolpath/grid.h>

// ROS messages
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

// Libraries
#include <Eigen/Dense>

using namespace smalldrop::smalldrop_vision;
using namespace smalldrop::smalldrop_segmentation;
using namespace smalldrop::smalldrop_toolpath;

/**
 * Global variables
 *********************************************************************************************/

pose_t initial_pose;
pose_t final_pose;
pose_t center;
bool has_camera = false;
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
unsigned int offset = 10;
unsigned int image_width = 640;
unsigned int image_height = 480;
double pose_z = 0.1;

/**
 * Function prototypes
 *********************************************************************************************/

void currentPoseCallback(geometry_msgs::Pose::ConstPtr msg);
bool processCmdArgs(int argc, char **argv);
void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform);

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

  camera_topics_t camera_topics;
  camera_topics.rgb_info_topic = "/smalldrop/vision/camera/color/camera_info";
  camera_topics.rgb_image_topic = "/smalldrop/vision/camera/color/image_raw";
  // camera_topics.ir1_info_topic = "/smalldrop/vision/camera/ir/camera_info";
  // camera_topics.ir1_image_topic = "/smalldrop/vision/camera/ir/image_raw";
  // camera_topics.ir2_info_topic = "/smalldrop/vision/camera/ir2/camera_info";
  // camera_topics.ir2_image_topic = "/smalldrop/vision/camera/ir2/image_raw";
  camera_topics.ir1_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir1_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.ir2_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir2_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.depth_info_topic = "/smalldrop/vision/camera/depth/camera_info";
  camera_topics.depth_image_topic = "/smalldrop/vision/camera/depth/image_raw";
  camera_topics.rgb_pcloud_topic = "/smalldrop/vision/camera/depth/color/points";

  tf::TransformListener tf_listener;
  Eigen::Matrix4d transform;
  transform.setIdentity();

  CameraD415 cam(true, camera_topics);

  // process command line arguments
  // change the final_pose or center pose depending on the arguments
  if (!processCmdArgs(argc, argv)) 
  {
    return 0;
  }

  if (has_camera)
  {
    int pcloud_size = 0;
    int im_w = 0;
    int im_h = 0;
    cam.turnOn();

    // Process current pose callback
    ros::Rate r(1);
    while (!has_pose && pcloud_size == 0 && im_w == 0 && im_h == 0)
    {
      PointCloud pc = cam.getPointCloud();
      sensor_msgs::Image img = cam.getRGBImage();
      pcloud_size = pc.width;
      im_w = img.width;
      im_h = img.height;

      if (!has_pose)
        std::cout << "Waiting for pose!" << std::endl;

      if (pcloud_size == 0)
        std::cout << "Waiting for point cloud!" << std::endl;
      else
        std::cout << "point cloud size: " << pcloud_size << std::endl;

      if (im_w == 0 || im_h == 0)
        std::cout << "Waiting for rgb image!" << std::endl;
      else
        std::cout << "rgb image size: " << im_w << "x" << im_h << std::endl;

      ros::spinOnce();
      r.sleep();
    }
  } else {
    ros::Rate wait(1);
    while(!has_pose)
    {
      ros::spinOnce();
      wait.sleep();
    }
  }

  // We have the initial pose. Set final and center poses
  final_pose = initial_pose;
  center = initial_pose;

  ros::Rate rate(freq);
  ros::Rate img_rate(1);
  ros::Time ts, te;
  double time_elapsed;

  poses_t traj;
  PATH_PLANE path_plane = PATH_PLANE::XY;
  
  if (plane.compare("xz") == 0)
    path_plane = PATH_PLANE::XZ;
  if (plane.compare("yz") == 0)
    path_plane = PATH_PLANE::YZ;
  
  TrajectoryPlanner pl(ttime, freq, PLAN_MODE::POLY3);
  // TrajectoryPlanner pl(ttime, freq, PLAN_MODE::LSPB);
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
  else if (path_type.compare("zigzag") == 0)
  {
    IMAGE_AXIS axis = IMAGE_AXIS::X;
    if (!plane.compare("xz") == 0 && !plane.compare("xy") == 0)
      axis = IMAGE_AXIS::Y;

    if (has_camera)
    {
      double distance = 0.702;
      double wsp_w = 0.838 * distance; 
      double wsp_h = 0.75 * wsp_w;
      img_wsp_calibration_t calibration_data = {
        .img_width = image_width,
        .img_height = image_height,
        .wsp_x_min = -wsp_w/2,
        .wsp_x_max = wsp_w/2,
        .wsp_y_min = -wsp_h/2,
        .wsp_y_max = wsp_h/2
      };

      getTransformToBase(tf_listener, transform);
      WSegmentCamBinary wseg(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);

      points_t contour = wseg.getWoundSegmentationPointsContour(0);
      poses_t poses_contour_region = wseg.getWoundSegmentationPosesContourRegion(0);
      int contour_size = contour.size();

      // If any error happens and the contour is empty
      // Spin once and try to get the contour again
      while(contour_size == 0)
      {
        ros::spinOnce();
        img_rate.sleep();

        wseg = WSegmentCamBinary(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);
        contour = wseg.getWoundSegmentationPointsContour(0);
        poses_contour_region = wseg.getWoundSegmentationPosesContourRegion(0);
        contour_size = contour.size();
      }

      ZigZag zigzag(contour, poses_contour_region, transform, offset, axis, calibration_data);
      Line line(initial_pose, zigzag.poses()[0]);
      std::vector<Path> paths = {line, zigzag};
      Trajectory t(pl.plan(paths));
      traj = t.poses();
    } else
    {
      img_wsp_calibration_t calibration_data = {
        .img_width = image_width,
        .img_height = image_height,
        .wsp_x_min = 0,
        .wsp_x_max = 1.0,
        .wsp_y_min = -0.5,
        .wsp_y_max = 0.5
      };

      std::stringstream path;
      path << ros::package::getPath("smalldrop_segmentation") << "/data/segmentation_points.dat";
      std::string filepath = path.str();
      WSegmentCoManipConvexHull wseg(filepath, calibration_data);
      points_t contour = wseg.getWoundSegmentationPointsContour(0);

      ZigZag zigzag(contour, offset, axis, pose_z, calibration_data);
      Line line(initial_pose, zigzag.poses()[0]);
      std::vector<Path> paths = {line, zigzag};
      Trajectory t(pl.plan(paths));
      traj = t.poses();
    }
  }
  else if (has_camera && path_type.compare("parallel_lines") == 0)
  {
    double distance = 0.702;
    double wsp_w = 0.838 * distance; 
    double wsp_h = 0.75 * wsp_w;
    img_wsp_calibration_t calibration_data = {
      .img_width = image_width,
      .img_height = image_height,
      .wsp_x_min = -wsp_w/2,
      .wsp_x_max = wsp_w/2,
      .wsp_y_min = -wsp_h/2,// + 0.075,
      .wsp_y_max = wsp_h/2// + 0.075
    };
    getTransformToBase(tf_listener, transform);
    WSegmentCamBinary wseg(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);

    points_t contour = wseg.getWoundSegmentationPointsContour(0);
    poses_t poses_contour_region = wseg.getWoundSegmentationPosesContourRegion(0);

    IMAGE_AXIS axis = IMAGE_AXIS::X;
    if (!plane.compare("xz") == 0 && !plane.compare("xy") == 0)
      axis = IMAGE_AXIS::Y;

    ParallelLines parallel_lines(contour, poses_contour_region, transform, offset, axis, calibration_data);
    Line line(initial_pose, parallel_lines.poses()[0]);
    std::vector<Path> paths = {line, parallel_lines};
    Trajectory t(pl.plan(paths));
    traj = t.poses();
  }
  else if (has_camera && path_type.compare("grid") == 0)
  {
    double distance = 0.702;
    double wsp_w = 0.838 * distance; 
    double wsp_h = 0.75 * wsp_w;
    img_wsp_calibration_t calibration_data = {
      .img_width = image_width,
      .img_height = image_height,
      .wsp_x_min = -wsp_w/2,
      .wsp_x_max = wsp_w/2,
      .wsp_y_min = -wsp_h/2,// + 0.075,
      .wsp_y_max = wsp_h/2// + 0.075
    };
    getTransformToBase(tf_listener, transform);
    WSegmentCamBinary wseg(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);

    points_t contour = wseg.getWoundSegmentationPointsContour(0);
    poses_t poses_contour_region = wseg.getWoundSegmentationPosesContourRegion(0);

    Grid grid(contour, poses_contour_region, transform, offset, offset, calibration_data);
    Line line(initial_pose, grid.poses()[0]);
    std::vector<Path> paths = {line, grid};
    Trajectory t(pl.plan(paths));
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

  if (has_camera)
    cam.turnOff();

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
  const char* const short_opts = ":csmrtfp:x:y:z:q:u:v:w:T:L:F:R:E:I:O:P:";
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
    {"offset", required_argument, nullptr, 'O'},
    {"help", no_argument, nullptr, 'h'},
    {nullptr, no_argument, nullptr, 0}
  };

  while ((opt = getopt_long(argc, argv, short_opts, long_opts, nullptr)) != -1)
  {
    switch (opt)
    {
      case 'c':
        has_camera = true;
        break;
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
        else if (path_type == "zigzag")
          pose_z = std::stod(optarg);
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
      case 'O':
        offset = std::stoi(optarg);
        break;
      case 'P':
        plane = static_cast<std::string>(optarg);
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "execute_trajectory -csmrtf -p <path_type> -P <trajectory_plane> -T <trajectory_time> -L <number_loops> \
        --freq <frequency> --radius <radius> --eradius <external_radius> --iradius <internal_radius> \
        -x <pos_x> -y <pos_y> -z <pos_z> --rx <orient_x> --ry <orient_y> --rz <orient_z> --rw <orient_w> --offset <toopath_offset>" << std::endl; 
        std::cout << "c: robot has camera" << std::endl; 
        std::cout << "s: show trajectory size" << std::endl; 
        std::cout << "m: show trajectory markers" << std::endl; 
        std::cout << "r: send trajectory to robot" << std::endl; 
        std::cout << "t: show trajectory planning time" << std::endl; 
        std::cout << "f: show trajectory tf" << std::endl;
        std::cout << "p: path type: line, circle, circular_spiral, zigzag, parallel_lines, grid" << std::endl;
        std::cout << "O: Toolpath offset" << std::endl;
        std::cout << "P: Trajectory plane: xy, xz, yz" << std::endl;
        return false;
        break;
    }
  }
  return true;
}

void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform)
{
  tf::StampedTransform transformStamped;
  try {
    tf_listener.lookupTransform("panda_link0", "camera_depth_optical_frame", ros::Time(0), transformStamped);
    
    Eigen::Vector4d t_vector(transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1);
    transform.col(3) << transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1;
    transform.col(0) << transformStamped.getBasis().getColumn(0)[0], transformStamped.getBasis().getColumn(0)[1], transformStamped.getBasis().getColumn(0)[2], 0;
    transform.col(1) << transformStamped.getBasis().getColumn(1)[0], transformStamped.getBasis().getColumn(1)[1], transformStamped.getBasis().getColumn(1)[2], 0;
    transform.col(2) << transformStamped.getBasis().getColumn(2)[0], transformStamped.getBasis().getColumn(2)[1], transformStamped.getBasis().getColumn(2)[2], 0;

    // std::cout << transform << std::endl;
  } catch (tf2::TransformException &ex) {
    return;
  }
}