#include <ros/package.h>
#include <ros/ros.h>
#include <trajectory_planner/cartesian_space_trajectory_planner.h>
#include <fstream>
#include <iostream>
#include <string>

using namespace trajectory_planner;

const std::string cartesian_poly3_param_prefix = "/cartesian_trajectory_profiles_plot/cartesian_space/poly3/";
const std::string cartesian_poly3c_param_prefix = "/cartesian_trajectory_profiles_plot/cartesian_space/poly3c/";
const std::string cartesian_poly3c_vias_param_prefix = "/cartesian_trajectory_profiles_plot/cartesian_space/poly3c_vias/";
const std::string cartesian_lspb_param_prefix = "/cartesian_trajectory_profiles_plot/cartesian_space/lspb/";

std::string profile = "";
std::vector<double> pose0;
std::vector<double> posef;
std::vector<double> velocity0;
std::vector<double> velocityf;
std::vector<std::vector<double>> poses;
std::vector<double> poses_x, poses_y, poses_z;
std::vector<double> times;
double acceleration;
double t0;
double tf;
double freq;
bool active;

bool check_params(ros::NodeHandle nh)
{
  if (nh.hasParam(cartesian_poly3_param_prefix + "active") &&
      nh.getParam(cartesian_poly3_param_prefix + "active", active) && active)
  {
    nh.getParam(cartesian_poly3_param_prefix + "pose0", pose0);
    nh.getParam(cartesian_poly3_param_prefix + "posef", posef);
    nh.getParam(cartesian_poly3_param_prefix + "t0", t0);
    nh.getParam(cartesian_poly3_param_prefix + "tf", tf);
    nh.getParam(cartesian_poly3_param_prefix + "frequency", freq);
    profile = "poly3";
    return true;
  }
  else if (nh.hasParam(cartesian_poly3c_param_prefix+"active") && nh.getParam(cartesian_poly3c_param_prefix+"active",
  active) && active)
  {
    nh.getParam(cartesian_poly3c_param_prefix+"pose0", pose0);
    nh.getParam(cartesian_poly3c_param_prefix+"posef", posef);
    nh.getParam(cartesian_poly3c_param_prefix+"velocity0", velocity0);
    nh.getParam(cartesian_poly3c_param_prefix+"velocityf", velocityf);
    nh.getParam(cartesian_poly3c_param_prefix+"t0", t0);
    nh.getParam(cartesian_poly3c_param_prefix+"tf", tf);
    nh.getParam(cartesian_poly3c_param_prefix+"frequency", freq);
    profile = "poly3c";
    return true;
  }
  else if (nh.hasParam(cartesian_poly3c_vias_param_prefix+"active") &&
  nh.getParam(cartesian_poly3c_vias_param_prefix+"active", active) && active)
  {
    nh.getParam(cartesian_poly3c_vias_param_prefix+"poses/x", poses_x);
    nh.getParam(cartesian_poly3c_vias_param_prefix+"poses/y", poses_y);
    nh.getParam(cartesian_poly3c_vias_param_prefix+"poses/z", poses_z);
    nh.getParam(cartesian_poly3c_vias_param_prefix+"times", times);
    nh.getParam(cartesian_poly3c_vias_param_prefix+"frequency", freq);
    tf = times.back();
    poses.resize(3);
    poses[0] = poses_x;
    poses[1] = poses_y;
    poses[2] = poses_z;
    profile = "poly3c_vias";
    return true;
  }
  else if (nh.hasParam(cartesian_lspb_param_prefix+"active") &&
  nh.getParam(cartesian_lspb_param_prefix+"active", active) && active)
  {
    nh.getParam(cartesian_lspb_param_prefix+"pose0", pose0);
    nh.getParam(cartesian_lspb_param_prefix+"posef", posef);
    nh.getParam(cartesian_lspb_param_prefix+"acceleration", acceleration);
    nh.getParam(cartesian_lspb_param_prefix+"t0", t0);
    nh.getParam(cartesian_lspb_param_prefix+"tf", tf);
    nh.getParam(cartesian_lspb_param_prefix+"frequency", freq);
    profile = "lspb";
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_trajectory_profiles_plot");
  ros::NodeHandle nh;

  if (!check_params(nh))
  {
    std::cout << "No valid profile found! Exiting." << std::endl;
    return 0;
  }

  std::cout << "Running profile: " << profile << std::endl;

  std::vector<std::vector<std::vector<double>>> cartesian;
  std::vector<double> time_v;
  double t = t0;

  ros::Rate r(freq);
  while (ros::ok() && t <= tf + 0.0001)
  {
    if (profile.compare("poly3") == 0)
      cartesian.push_back(CartesianSpaceTrajectoryPlanner::poly3(pose0, posef, t0, tf, t));
    else if (profile.compare("poly3c") == 0)
      cartesian.push_back(CartesianSpaceTrajectoryPlanner::poly3c(pose0, posef, velocity0, velocityf, t0, tf, t));
    else if (profile.compare("poly3c_vias") == 0)
      cartesian.push_back(CartesianSpaceTrajectoryPlanner::poly3c_vias(poses, times, t));
    else if (profile.compare("lspb") == 0)
      cartesian.push_back(CartesianSpaceTrajectoryPlanner::lspb(pose0, posef, acceleration, t0, tf, t));

    time_v.push_back(t);
    t += 1 / freq;
    r.sleep();
  }

  std::fstream fh;
  std::string path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools";
  fh.open(path + "/data/cartesian_profile.dat", std::fstream::out);

  if (fh.is_open())
  {
    for (size_t i = 0; i < time_v.size(); i++)
    {
      fh << time_v[i] << " " << cartesian[i][0][0] << " " << cartesian[i][1][0] << " " << cartesian[i][2][0] << " "
         << cartesian[i][0][1] << " " << cartesian[i][1][1] << " " << cartesian[i][2][1] << " " 
         << cartesian[i][0][2] << " " << cartesian[i][1][2] << " " << cartesian[i][2][2] << " "
         << cartesian[i][0][3] << " " << cartesian[i][1][3] << " " << cartesian[i][2][3] << " "
         << cartesian[i][0][4] << " " << cartesian[i][1][4] << " " << cartesian[i][2][4] << " "
         << cartesian[i][0][5] << " " << cartesian[i][1][5] << " " << cartesian[i][2][5] << " "
         << cartesian[i][0][6] << " " << cartesian[i][1][6] << " " << cartesian[i][2][6] << "\n";
    }
  }
  else
  {
    ROS_ERROR("The files were not properly opened!");
  }
  fh.close();

  std::cout << "Finished running " << profile << " profile." << std::endl;

  return 0;
}
