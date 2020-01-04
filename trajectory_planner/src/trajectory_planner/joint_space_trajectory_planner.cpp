
#include <trajectory_planner/joint_space_trajectory_planner.h>
#include <cmath>

namespace trajectory_planner
{
  // 3rd order polynomial with zero initial and final joints speeds
  Eigen::Vector3d JointSpaceTrajectoryPlanner::poly3(const double theta0, const double thetaf, 
                                            const double tf, const double t0, const double t)
  {
    double a0 = theta0;
    double a1 = 0;
    double a2 = (3 / pow(tf, 2)) * (thetaf - theta0);
    double a3 = - (2 / pow(tf, 3)) * (thetaf - theta0);

    double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3);
    double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2);
    double theta_dd = 2 * a2 + 6 * a3 * (t - t0);

    return Eigen::Vector3d(theta, theta_d, theta_dd);
  }
  
  // 3rd order polynomial with zero initial and final joints speeds
  Eigen::Vector3d JointSpaceTrajectoryPlanner::poly3c(const double theta0, const double thetaf, 
                                            const double theta0_d, const double thetaf_d, 
                                            const double tf, const double t0, const double t)
  {
    double a0 = theta0;
    double a1 = theta0_d;
    double a2 = (3 / pow(tf, 2)) * (thetaf - theta0) - (2 / tf) * theta0_d - (1 / tf) * thetaf_d;
    double a3 = - (2 / pow(tf, 3)) * (thetaf - theta0) + (1 / pow(tf, 2)) * (thetaf_d + theta0_d);

    double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3);
    double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2);
    double theta_dd = 2 * a2 + 6 * a3 * (t - t0);

    return Eigen::Vector3d(theta, theta_d, theta_dd);
  }

  Eigen::Vector3d JointSpaceTrajectoryPlanner::poly5c(const double theta0, const double thetaf, 
                                                    const double theta0_d, const double thetaf_d, 
                                                    const double theta0_dd, const double thetaf_dd, 
                                                    const double tf, const double t0, const double t)
  {
    double a0 = theta0;
    double a1 = theta0_d;
    double a2 = theta0_dd / 2;
    double a3 = (20 * thetaf - 20 * theta0 - (8 * thetaf_d + 12 * theta0_d) * tf - (3 * theta0_dd - thetaf_dd) * pow(tf, 2)) / (2 * pow(tf, 3));
    double a4 = (30 * theta0 - 30 * thetaf + (14 * thetaf_d + 16 * theta0_d) * tf + (3 * theta0_dd - 2 * thetaf_dd) * pow(tf, 2)) / (2 * pow(tf, 4));
    double a5 = (12 * thetaf - 12 * theta0 - (6 * thetaf_d + 6 * theta0_d) * tf - (theta0_dd - thetaf_dd) * pow(tf, 2)) / (2 * pow(tf, 5));

    double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3) + a4 * pow(t - t0, 4) + a5 * pow(t - t0, 5);
    double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2) + 4 * a4 * pow(t - t0, 3) + 5 * a5 * pow(t - t0, 4);
    double theta_dd = 2 * a2 + 6 * a3 * (t - t0) + 12 * a4 * pow(t - t0, 2) + 20 * a5 * pow(t - t0, 3) ;

    return Eigen::Vector3d(theta, theta_d, theta_dd);
  }
} // namespace trajectory_planner