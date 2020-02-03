#ifndef _CARTESIAN_SPACE_TRAJECTORY_PLANNER
#define _CARTESIAN_SPACE_TRAJECTORY_PLANNER

#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>
#include <vector>

namespace trajectory_planner
{
class CartesianSpaceTrajectoryPlanner
{
private:
  // Calculate via velocity based on heuristic
  static double via_velocity(const double pos_before, const double pos, const double pos_after, const double t_before,
                             const double t, const double t_after);

public:
  // 3rd order polynomial with zero initial and final velocities
  static std::vector<std::vector<double>> poly3(const std::vector<double> pose0, const std::vector<double> posef,
                                                const double t0, const double tf, const double t);
  // 3rd order polynomial with non-zero initial and final velocities
  static std::vector<std::vector<double>> poly3c(const std::vector<double> pose0, const std::vector<double> posef,
                                                 const std::vector<double> velocity0,
                                                 const std::vector<double> velocityf, const double t0, const double tf,
                                                 const double t);
  // 3rd order polynomial with via points and velocity heuristics
  static std::vector<std::vector<double>> poly3c_vias(const std::vector<std::vector<double>> poses,
                                                      const std::vector<double> times, const double t);
  // Linear segment with parabolic blends
  static std::vector<std::vector<double>> lspb(const std::vector<double> pose0, const std::vector<double> posef, const double accel, const double t0,
                                               const double tf, const double t);

  // 3rd order polynomial with zero initial and final velocities, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3p(const geometry_msgs::Pose pose0, const geometry_msgs::Pose posef, const double t0,
                                    const double tf, const double t);
  // 3rd order polynomial with non-zero initial and final velocities, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3pc(const geometry_msgs::Pose pose0, const geometry_msgs::Pose posef,
                                     const std::vector<double> velocity0, const std::vector<double> velocityf,
                                     const double t0, const double tf, const double t);
  // 3rd order polynomial with via points and velocity heuristics, that uses geometry_msgs::Pose
  static geometry_msgs::Pose poly3pc_vias(const std::vector<geometry_msgs::Pose> poses,
                                                      const std::vector<double> times, const double t);                                  
  // Linear segment with parabolic blends, that uses geometry_msgs::Pose
  static geometry_msgs::Pose lspbp(const geometry_msgs::Pose pose0, const geometry_msgs::Pose posef, const double accel, const double t0,
                                               const double tf, const double t);
};
}  // namespace trajectory_planner

#endif  // _CARTESIAN_SPACE_TRAJECTORY_PLANNER