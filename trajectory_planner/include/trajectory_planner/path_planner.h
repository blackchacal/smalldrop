#ifndef _PATH_PLANNER
#define _PATH_PLANNER

#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <vector>

namespace trajectory_planner
{
enum class PATH_TYPE
{
  LINE,
  CIRCLE,
  ELLIPSE,
  SQUARE,
  RECTANGLE,
  CIRCULAR_SPIRAL,
  RECTANGULAR_SPIRAL
};

enum class PATH_PLANE
{
  XY,
  XZ,
  YZ
};

class PathPlanner
{
private:
  /* data */
public:
  // Plan a linear path between two poses
  static std::vector<geometry_msgs::Pose> line_path(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f);
  // Plan a circular path centered on a specific point
  static std::vector<geometry_msgs::Pose> circle_path(const geometry_msgs::Pose pose_i,
                                                      const geometry_msgs::Pose center, const double radius,
                                                      const unsigned int loops, const unsigned int n,
                                                      const PATH_PLANE plane = PATH_PLANE::XY);
  // Plan a circular spiral path centered on a specific point
  static std::vector<geometry_msgs::Pose> circular_spiral_path(const geometry_msgs::Pose pose_i, const double eradius,
                                                               const double iradius, const unsigned int n,
                                                               const unsigned int loops = 1,
                                                               const PATH_PLANE plane = PATH_PLANE::XY);
};
}  // namespace trajectory_planner

#endif  // _PATH_PLANNER