#ifndef _CARTESIAN_SPACE_TRAJECTORY_PLANNER
#define _CARTESIAN_SPACE_TRAJECTORY_PLANNER

#include <geometry_msgs/Pose.h>
#include <trajectory_planner/path_planner.h>
#include <Eigen/Dense>
#include <vector>

namespace trajectory_planner
{
enum class PLAN_MODE
{
  POLY3,
  POLY3_VIAS,
  LSPB
};

class TaskTrajectoryPlanner
{
private:
  static const unsigned int line_npoints = 10;
  static const unsigned int circle_npoints = 20;
  static const unsigned int ellipse_npoints = 10;
  static const unsigned int spiral_npoints = 20;
  static const unsigned int lspb_accel = 30;

  // Calculate via velocity based on heuristic
  static double via_velocity(const double pos_before, const double pos, const double pos_after, const double t_before,
                             const double t, const double t_after);

public:
  // 3rd order polynomial with zero initial and final velocities
  static std::vector<std::vector<double>> poly3(const std::vector<double> pose_i, const std::vector<double> pose_f,
                                                const double t0, const double tf, const double t);
  // 3rd order polynomial with non-zero initial and final velocities
  static std::vector<std::vector<double>> poly3c(const std::vector<double> pose_i, const std::vector<double> pose_f,
                                                 const std::vector<double> velocity0,
                                                 const std::vector<double> velocityf, const double t0, const double tf,
                                                 const double t);
  // 3rd order polynomial with via points and velocity heuristics
  static std::vector<std::vector<double>> poly3c_vias(const std::vector<std::vector<double>> poses,
                                                      const std::vector<double> times, const double t);
  // Linear segment with parabolic blends
  static std::vector<std::vector<double>> lspb(const std::vector<double> pose_i, const std::vector<double> pose_f,
                                               const double accel, const double t0, const double tf, const double t);

  // 3rd order polynomial with zero initial and final velocities, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3p(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f, const double t0,
                                    const double tf, const double t);
  // 3rd order polynomial with non-zero initial and final velocities, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3pc(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f,
                                     const std::vector<double> velocity0, const std::vector<double> velocityf,
                                     const double t0, const double tf, const double t);
  // 3rd order polynomial with via points and velocity heuristics, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3pc_vias(const std::vector<geometry_msgs::Pose> poses, const std::vector<double> times,
                                          const double t);
  // Linear segment with parabolic blends, that uses geometry_msgs::Pose
  static geometry_msgs::Pose lspbp(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f,
                                   const double accel, const double t0, const double tf, const double t);
  // General trajectory planner for all the paths
  static std::vector<geometry_msgs::Pose> plan_trajectory(std::vector<geometry_msgs::Pose> path, const double duration,
                                                          const double frequency, const PLAN_MODE mode);
  
  // Plans a linear path trajectory with a certain duration, frequency and planning mode.
  static std::vector<geometry_msgs::Pose> linear_trajectory(const geometry_msgs::Pose pose_i,
                                                            const geometry_msgs::Pose pose_f, const double duration,
                                                            const double frequency,
                                                            const PLAN_MODE mode = PLAN_MODE::POLY3);
  // Plans a circular path trajectory with a certain duration, frequency and planning mode.
  static std::vector<geometry_msgs::Pose> circular_trajectory(const geometry_msgs::Pose pose_i,
                                                              const geometry_msgs::Pose center, const double radius,
                                                              const double duration, const double frequency,
                                                              const unsigned int loops = 1,
                                                              const PATH_PLANE plane = PATH_PLANE::XY,
                                                              const PLAN_MODE mode = PLAN_MODE::POLY3);
  // Plans a circular spiral path trajectory with a certain duration, frequency and planning mode.
  static std::vector<geometry_msgs::Pose> circular_spiral_trajectory(const geometry_msgs::Pose pose_i,
                                                                     const double eradius, const double iradius,
                                                                     const double duration, const double frequency,
                                                                     const unsigned int loops = 2, const PATH_PLANE plane = PATH_PLANE::XY,
                                                                     const PLAN_MODE mode = PLAN_MODE::POLY3);
};
}  // namespace trajectory_planner

#endif  // _CARTESIAN_SPACE_TRAJECTORY_PLANNER