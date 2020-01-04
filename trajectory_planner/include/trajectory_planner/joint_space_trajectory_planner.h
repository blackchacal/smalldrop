#ifndef _JOINT_SPACE_TRAJECTORY_PLANNER
#define _JOINT_SPACE_TRAJECTORY_PLANNER

#include <Eigen/Dense>

namespace trajectory_planner
{
  class JointSpaceTrajectoryPlanner
  {
    private:
      /* data */
    public:
      // 3rd order polynomial with zero initial and final joints velocities
      Eigen::Vector3d poly3(const double theta0, const double thetaf, 
                            const double tf, const double t0, const double t);
      // 3rd order polynomial with non-zero initial and final joints velocities
      Eigen::Vector3d poly3c(const double theta0, const double thetaf, const double theta0_d, 
                            const double thetaf_d, const double tf, const double t0, const double t);
      // 3rd order polynomial with non-zero initial and final joints velocities and accelerations
      Eigen::Vector3d poly5c(const double theta0, const double thetaf, const double theta0_d, 
                            const double thetaf_d, const double theta0_dd, const double thetaf_dd, 
                            const double tf, const double t0, const double t);
  };
} // namespace trajectory_planner

#endif // _JOINT_SPACE_TRAJECTORY_PLANNER