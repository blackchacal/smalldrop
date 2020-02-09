#ifndef _JOINT_SPACE_TRAJECTORY_PLANNER
#define _JOINT_SPACE_TRAJECTORY_PLANNER

#include <Eigen/Dense>
#include <vector>

namespace trajectory_planner
{
class JointTrajectoryPlanner
{
private:
  // Calculate via velocity based on heuristic
  static double via_velocity(const double theta_before, const double theta, const double theta_after,
                             const double t_before, const double t, const double t_after);

public:
  // 3rd order polynomial with zero initial and final joints velocities
  static Eigen::Vector3d poly3(const double theta0, const double thetaf, const double t0, const double tf,
                               const double t);
  // 3rd order polynomial with non-zero initial and final joints velocities
  static Eigen::Vector3d poly3c(const double theta0, const double thetaf, const double theta0_d, const double thetaf_d,
                                const double t0, const double tf, const double t);
  // 3rd order polynomial with via points and velocity heuristics
  static Eigen::Vector3d poly3c_vias(const std::vector<double> thetas, const std::vector<double> times, const double t);
  // 5th order polynomial with non-zero initial and final joints velocities and accelerations
  static Eigen::Vector3d poly5c(const double theta0, const double thetaf, const double theta0_d, const double thetaf_d,
                                const double theta0_dd, const double thetaf_dd, const double t0, const double tf,
                                const double t);
  // Linear segment with parabolic blends
  static Eigen::Vector3d lspb(const double theta0, const double thetaf, const double thetab_dd, const double t0,
                              const double tf, const double t);
  // Linear segment with parabolic blends and via points
  static Eigen::Vector3d lspb_vias(const std::vector<double> thetas, const std::vector<double> times,
                                   const double thetab_dd, const double t);
};
}  // namespace trajectory_planner

#endif  // _JOINT_SPACE_TRAJECTORY_PLANNER