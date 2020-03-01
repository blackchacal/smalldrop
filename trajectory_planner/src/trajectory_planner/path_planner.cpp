#include <trajectory_planner/path_planner.h>
#include <cmath>

namespace trajectory_planner
{
  // Plan a linear path between two poses
  std::vector<geometry_msgs::Pose> PathPlanner::line_path(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f)
  {
    std::vector<geometry_msgs::Pose> path;
    double t = 0;
    while (t < 1.0001)
    {
      geometry_msgs::Pose pose;
      pose.position.x = (1-t) * pose_i.position.x + t * pose_f.position.x;
      pose.position.y = (1-t) * pose_i.position.y + t * pose_f.position.y;
      pose.position.z = (1-t) * pose_i.position.z + t * pose_f.position.z;
      Eigen::Quaterniond qres;
      Eigen::Quaterniond qi(pose_i.orientation.w, pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z);
      Eigen::Quaterniond qf(pose_f.orientation.w, pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z);
      qres = qi.slerp(t, qf);
      pose.orientation.x = qres.x();
      pose.orientation.y = qres.y();
      pose.orientation.z = qres.z();
      pose.orientation.w = qres.w();
      path.push_back(pose);
      t++;
    }
    return path;
  }

  // Plan a circular path centered on a specific point
  std::vector<geometry_msgs::Pose> PathPlanner::circle_path(const geometry_msgs::Pose pose_i, 
                                                            const geometry_msgs::Pose center, const double radius,
                                                            const unsigned int n, const unsigned int loops,
                                                            const PATH_PLANE plane)
  {
    std::vector<geometry_msgs::Pose> path;

    // Add movement from actual pose to circle start pose
    geometry_msgs::Pose start_pose; 
    switch (plane)
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
    start_pose.orientation.x = pose_i.orientation.x;
    start_pose.orientation.y = pose_i.orientation.y;
    start_pose.orientation.z = pose_i.orientation.z;
    start_pose.orientation.w = pose_i.orientation.w;
    path = PathPlanner::line_path(pose_i, start_pose);

    for (size_t i = 0; i < loops; i++)
    {
      double t = 0;
      double step = 1 / (double)n;
      while (t <= 1.0001)
      {
        geometry_msgs::Pose pose;
        switch (plane)
        {
        case PATH_PLANE::XZ:
          pose.position.x = radius * cos(2 * M_PI * t) + center.position.x;
          pose.position.y = center.position.y;
          pose.position.z = radius * sin(2 * M_PI * t) + center.position.z;
          break;
        case PATH_PLANE::YZ:
          pose.position.x = center.position.x;
          pose.position.y = radius * cos(2 * M_PI * t) + center.position.y;
          pose.position.z = radius * sin(2 * M_PI * t) + center.position.z;
          break;
        default:
          pose.position.x = radius * cos(2 * M_PI * t) + center.position.x;
          pose.position.y = radius * sin(2 * M_PI * t) + center.position.y;
          pose.position.z = center.position.z;
          break;
        }
        pose.orientation.x = pose_i.orientation.x;
        pose.orientation.y = pose_i.orientation.y;
        pose.orientation.z = pose_i.orientation.z;
        pose.orientation.w = pose_i.orientation.w;

        path.push_back(pose);
        t += step;
      }
    }
    return path;
  }   

  // Plan a circular spiral path centered on a specific point
  std::vector<geometry_msgs::Pose> PathPlanner::circular_spiral_path(const geometry_msgs::Pose pose_i,
                                                               const double eradius, const double iradius, 
                                                               const unsigned int loops, 
                                                               const unsigned int n, const PATH_PLANE plane)
  {
    std::vector<geometry_msgs::Pose> path;

    double t;
    double step = 1 / (double)n;
    double rstep = (eradius - iradius) / (double)loops;
    double sub_rstep = rstep / (double)n;
    double radius = eradius;
    double i = eradius;

    while (i > iradius + 0.0001)
    {
      t = 0;
      while (t <= 0.9999)
      {
        geometry_msgs::Pose pose;
        switch (plane)
        {
        case PATH_PLANE::XZ:
          pose.position.x = radius * cos(2 * M_PI * t) + pose_i.position.x - eradius;
          pose.position.y = pose_i.position.y;
          pose.position.z = radius * sin(2 * M_PI * t) + pose_i.position.z;
          break;
        case PATH_PLANE::YZ:
          pose.position.x = pose_i.position.x;
          pose.position.y = radius * cos(2 * M_PI * t) + pose_i.position.y - eradius;
          pose.position.z = radius * sin(2 * M_PI * t) + pose_i.position.z;
          break;
        default:
          pose.position.x = radius * cos(2 * M_PI * t) + pose_i.position.x - eradius;
          pose.position.y = radius * sin(2 * M_PI * t) + pose_i.position.y;
          pose.position.z = pose_i.position.z;
          break;
        }
        pose.orientation.x = pose_i.orientation.x;
        pose.orientation.y = pose_i.orientation.y;
        pose.orientation.z = pose_i.orientation.z;
        pose.orientation.w = pose_i.orientation.w;

        path.push_back(pose);
        t += step;
        radius -= sub_rstep;
      }
      i -= rstep;
    }
    return path;
  }                                                                                                                       
}