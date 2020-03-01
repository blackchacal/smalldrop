#include <ros/ros.h>
#include <panda_3dbioprint_robotic_system/trajectory_planner.h>
#include <visualization_msgs/Marker.h>
#include <panda_3dbioprint_robotic_system/SetPathType.h>

using namespace trajectory_planner;
using namespace robotic_system;

std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";
unsigned int imageWidth = 400;
unsigned int imageHeight = 400;
unsigned int freq = 100; // 100 Hz
unsigned int offset_x = 5;
unsigned int offset_y = 5;
double pose_z = 0.05;
double duration = 5;
robotic_system::PATH_TYPE path_type = robotic_system::PATH_TYPE::PARALLEL_LINES;
IMAGE_AXIS axis = IMAGE_AXIS::X;
PLAN_MODE mode = PLAN_MODE::POLY3;
geometry_msgs::Pose initial_pose;
bool has_pose = false;
std::string mode_str = "poly3";
std::string path_str = "parallel_lines";
std::string axis_str = "x";

// Function declarations
void currentPoseCallback(geometry_msgs::PoseConstPtr msg);
std::vector<geometry_msgs::Pose> genTrajectory(void);
void updateToopathViewer(ros::NodeHandle nh);
void addTrajectoryMarkers(ros::Publisher pub_markers, geometry_msgs::Pose pose_i, geometry_msgs::Pose pose_f, int marker_id);
void cleanTrajectoryMarkers(ros::Publisher pub_markers);
bool processCmdArgs(int argc, char **argv);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "trajectory_execution_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/desired_pose", 10);
  ros::Publisher pub_markers = nh.advertise<visualization_msgs::Marker>("/panda_3dbioprint_simulation/trajectory_markers", 10);
  ros::Subscriber sub = nh.subscribe("/cartesian_impedance_controller/current_pose", 10, currentPoseCallback);


  // Process current pose callback
  ros::Rate r(10);
  while (ros::ok() && !has_pose)
  {
    // Delete previous trajectory markers
    cleanTrajectoryMarkers(pub_markers);
    ros::spinOnce();
    r.sleep();
  }

  // process command line arguments
  if (!processCmdArgs(argc, argv)) 
    return 0;

  // Call service to update the toolpath visualisation of rviz
  updateToopathViewer(nh);

  // Generate trajectory
  std::vector<geometry_msgs::Pose> trajectory;
  trajectory = genTrajectory();

  ros::Rate rate(freq);
  for (size_t i = 0; i < trajectory.size(); i++)
  {
    geometry_msgs::Pose p = trajectory[i];
    if (i < trajectory.size()-1)
      addTrajectoryMarkers(pub_markers, trajectory[i], trajectory[i+1], i);

    pub.publish(p);

    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}

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

std::vector<geometry_msgs::Pose> genTrajectory(void)
{
  TrajectoryPlanner pl = TrajectoryPlanner(imageWidth, imageHeight, 0, 1, -0.5, 0.5);
  std::vector<geometry_msgs::Pose> trajectory;
  std::vector<geometry_msgs::Pose> go_to_start;
  std::vector<geometry_msgs::Pose> tool_trajectory;
  std::vector<geometry_msgs::Pose> homing;

  tool_trajectory = pl.planTrajectory(filepath, duration, freq, path_type, offset_x, offset_y, pose_z, axis, mode);
  go_to_start = TaskTrajectoryPlanner::linear_trajectory(initial_pose, tool_trajectory.front(), duration, freq, mode);
  homing = TaskTrajectoryPlanner::linear_trajectory(tool_trajectory.back(), initial_pose, duration, freq, mode);
  trajectory.insert(trajectory.end(), go_to_start.begin(), go_to_start.end());
  trajectory.insert(trajectory.end(), tool_trajectory.begin(), tool_trajectory.end());
  trajectory.insert(trajectory.end(), homing.begin(), homing.end());

  return trajectory;
}

void updateToopathViewer(ros::NodeHandle nh)
{
  ros::ServiceClient client = nh.serviceClient<panda_3dbioprint_robotic_system::SetPathType>("change_path_type");
  panda_3dbioprint_robotic_system::SetPathType srv;
  srv.request.path_type = path_str;
  srv.request.offset_x = offset_x;
  srv.request.offset_y = offset_y;
  srv.request.axis = axis_str;
  if (!client.call(srv))
    ROS_ERROR("Failed to call service change_path_type.");
}

void addTrajectoryMarkers(ros::Publisher pub_markers, geometry_msgs::Pose pose_i, geometry_msgs::Pose pose_f, int marker_id)
{
  std::vector<geometry_msgs::Point> points;
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  p1.x = pose_i.position.x;
  p1.y = pose_i.position.y;
  p1.z = pose_i.position.z;
  p2.x = pose_f.position.x;
  p2.y = pose_f.position.y;
  p2.z = pose_f.position.z;
  points.push_back(p1);
  points.push_back(p2);

  visualization_msgs::Marker marker;
  marker.header.frame_id = "panda_link0";
  marker.header.stamp = ros::Time();
  marker.ns = "panda_3dbioprint_simulation";
  marker.id = marker_id;
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

void cleanTrajectoryMarkers(ros::Publisher pub_markers)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "panda_link0";
  marker.header.stamp = ros::Time();
  marker.ns = "panda_3dbioprint_simulation";
  marker.action = visualization_msgs::Marker::DELETEALL;
  pub_markers.publish(marker);
}

bool processCmdArgs(int argc, char **argv)
{
  // Process command-line arguments
  int opt;
  const char* const short_opts = ":hx:y:z:d:m:p:a:";

  while ((opt = getopt(argc, argv, short_opts)) != -1)
  {
    switch (opt)
    {
      case 'x':
        offset_x = std::stoi(optarg);
        break;
      case 'y':
        offset_y = std::stoi(optarg);
        break;
      case 'z':
        pose_z = std::stod(optarg);
        break;
      case 'd':
        duration = std::stod(optarg);
        break;
      case 'm':
        mode_str = static_cast<std::string>(optarg);
        if (mode_str.compare("poly3") == 0)
          mode = PLAN_MODE::POLY3;
        else
          mode = PLAN_MODE::LSPB;
        break;
      case 'p':
        path_str = static_cast<std::string>(optarg);
        if (path_str.compare("zig_zag") == 0)
          path_type = robotic_system::PATH_TYPE::ZIG_ZAG;
        else if (path_str.compare("parallel_lines") == 0)
          path_type = robotic_system::PATH_TYPE::PARALLEL_LINES;
        else
          path_type = robotic_system::PATH_TYPE::GRID;
        break;
      case 'a':
        axis_str = static_cast<std::string>(optarg);
        if (axis_str.compare("x") == 0)
          axis = IMAGE_AXIS::X;
        else
          axis = IMAGE_AXIS::Y;
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "trajectory_execution_node -h -x <offset_x> -y <offset_y> -z <pose_z> \
        -d <duration> -m <planning_mode> -p <path_type> -a <axis>" << std::endl; 
        std::cout << "m: poly3 | lspb" << std::endl; 
        std::cout << "p: zig_zag | parallel | grid" << std::endl; 
        std::cout << "a: x | y" << std::endl; 
        return false;
        break;
    }
  }
  return true;
}