#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_planner/joint_space_trajectory_planner.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace trajectory_planner;

const std::string joint_poly3_param_prefix = "/joint_trajectory_profiles_plot/joint_space/poly3/";
const std::string joint_poly3c_param_prefix = "/joint_trajectory_profiles_plot/joint_space/poly3c/";
const std::string joint_poly5c_param_prefix = "/joint_trajectory_profiles_plot/joint_space/poly5c/";

std::string profile = "";
double theta0;
double thetaf;
double theta0_d;
double thetaf_d;
double theta0_dd;
double thetaf_dd;
double t0;
double tf;
double freq;
bool active;

bool check_params(ros::NodeHandle nh)
{
  if (nh.hasParam(joint_poly3_param_prefix+"active") && nh.getParam(joint_poly3_param_prefix+"active", active) && active)
  {
    nh.getParam(joint_poly3_param_prefix+"theta0", theta0);
    nh.getParam(joint_poly3_param_prefix+"thetaf", thetaf);
    nh.getParam(joint_poly3_param_prefix+"t0", t0);
    nh.getParam(joint_poly3_param_prefix+"tf", tf);
    nh.getParam(joint_poly3_param_prefix+"frequency", freq);
    profile = "poly3";
    return true;
  }
  else if (nh.hasParam(joint_poly3c_param_prefix+"active") && nh.getParam(joint_poly3c_param_prefix+"active", active) && active)
  {
    nh.getParam(joint_poly3c_param_prefix+"theta0", theta0);
    nh.getParam(joint_poly3c_param_prefix+"thetaf", thetaf);
    nh.getParam(joint_poly3c_param_prefix+"theta0_d", theta0_d);
    nh.getParam(joint_poly3c_param_prefix+"thetaf_d", thetaf_d);
    nh.getParam(joint_poly3c_param_prefix+"t0", t0);
    nh.getParam(joint_poly3c_param_prefix+"tf", tf);
    nh.getParam(joint_poly3c_param_prefix+"frequency", freq);
    profile = "poly3c";
    return true;
  }
  else if (nh.hasParam(joint_poly5c_param_prefix+"active") && nh.getParam(joint_poly5c_param_prefix+"active", active) && active)
  {
    nh.getParam(joint_poly5c_param_prefix+"theta0", theta0);
    nh.getParam(joint_poly5c_param_prefix+"thetaf", thetaf);
    nh.getParam(joint_poly5c_param_prefix+"theta0_d", theta0_d);
    nh.getParam(joint_poly5c_param_prefix+"thetaf_d", thetaf_d);
    nh.getParam(joint_poly5c_param_prefix+"theta0_dd", theta0_dd);
    nh.getParam(joint_poly5c_param_prefix+"thetaf_dd", thetaf_dd);
    nh.getParam(joint_poly5c_param_prefix+"t0", t0);
    nh.getParam(joint_poly5c_param_prefix+"tf", tf);
    nh.getParam(joint_poly5c_param_prefix+"frequency", freq);
    profile = "poly5c";
    return true;
  }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_trajectory_profiles_plot");
  ros::NodeHandle nh;

  if (!check_params(nh)) {
    std::cout << "No valid profile found! Exiting." << std::endl;
    return 0;
  }

  std::cout << "Running profile: " << profile << std::endl;

  JointSpaceTrajectoryPlanner pl = JointSpaceTrajectoryPlanner();
  std::vector<Eigen::Vector3d> joint;
  std::vector<double> time_v;
  double t = t0;

  ros::Rate r(freq);
  while (ros::ok() && t < tf)
  {
    if (profile.compare("poly3") == 0) {
      joint.push_back(pl.poly3(theta0, thetaf, tf, t0, t));
    }
    else if (profile.compare("poly3c") == 0) {
      joint.push_back(pl.poly3c(theta0, thetaf, theta0_d, thetaf_d, tf, t0, t));
    }
    else if (profile.compare("poly5c") == 0) {
      joint.push_back(pl.poly5c(theta0, thetaf, theta0_d, thetaf_d, theta0_dd, thetaf_dd, tf, t0, t));
    }  
    time_v.push_back(t);
    t += 1/freq;
    r.sleep();
  }

  std::fstream fh;
  std::string path = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools";
  fh.open(path+"/data/single_joint_profile.dat", std::fstream::out);

  if (fh.is_open()) 
  {
    for (size_t i = 0; i < time_v.size(); i++)
    {
      fh << time_v[i] << " " << joint[i](0) << " " << joint[i](1) << " " << joint[i](2) << "\n";
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
