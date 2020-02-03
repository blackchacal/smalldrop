#include <ros/ros.h>
#include <trajectory_planner/joint_space_trajectory_planner.h>
#include <cmath>

namespace trajectory_planner
{
// 3rd order polynomial with zero initial and final joints speeds
Eigen::Vector3d JointSpaceTrajectoryPlanner::poly3(const double theta0, const double thetaf, const double t0,
                                                   const double tf, const double t)
{
  double a0 = theta0;
  double a1 = 0;
  double a2 = (3 / pow(tf - t0, 2)) * (thetaf - theta0);
  double a3 = -(2 / pow(tf - t0, 3)) * (thetaf - theta0);

  double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3);
  double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2);
  double theta_dd = 2 * a2 + 6 * a3 * (t - t0);

  return Eigen::Vector3d(theta, theta_d, theta_dd);
}

// 3rd order polynomial with zero initial and final joints speeds
Eigen::Vector3d JointSpaceTrajectoryPlanner::poly3c(const double theta0, const double thetaf, const double theta0_d,
                                                    const double thetaf_d, const double t0, const double tf,
                                                    const double t)
{
  double a0 = theta0;
  double a1 = theta0_d;
  double a2 = (3 / pow(tf - t0, 2)) * (thetaf - theta0) - (2 / (tf - t0)) * theta0_d - (1 / (tf - t0)) * thetaf_d;
  double a3 = -(2 / pow(tf - t0, 3)) * (thetaf - theta0) + (1 / pow(tf - t0, 2)) * (thetaf_d + theta0_d);

  double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3);
  double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2);
  double theta_dd = 2 * a2 + 6 * a3 * (t - t0);

  return Eigen::Vector3d(theta, theta_d, theta_dd);
}

// Calculate via velocity based on heuristic
double JointSpaceTrajectoryPlanner::via_velocity(const double theta_before, const double theta,
                                                 const double theta_after, const double t_before, const double t,
                                                 const double t_after)
{
  double theta_i_d, theta_ii_d;

  theta_i_d = (theta - theta_before) / (t - t_before);  // Mean velocity before via point
  theta_ii_d = (theta_after - theta) / (t_after - t);   // Mean velocity after via point

  if ((theta_i_d * theta_ii_d) < 0)  // Mean velocities change signal at via point
    return 0.0;                      // Velocity at via point is zero
  else
    return 0.5 * (theta_ii_d + theta_i_d);  // Velocity at via point is the average of the mean velocities
}

// 3rd order polynomial with via points and velocity heuristics
Eigen::Vector3d JointSpaceTrajectoryPlanner::poly3c_vias(const std::vector<double> thetas,
                                                         const std::vector<double> times, const double t)
{
  if (thetas.size() != times.size())
  {
    ROS_ERROR("The thetas vector should be the same size as times vector.");
    return Eigen::Vector3d(-1, -1, -1);
  }

  double theta0_d, thetaf_d;

  for (size_t i = 1; i < times.size(); i++)
    if (t >= times[i - 1] && t <= times[i])
    {
      if (i == 1)
      {
        theta0_d = 0.0;
        thetaf_d = JointSpaceTrajectoryPlanner::via_velocity(thetas[i - 1], thetas[i], thetas[i + 1], times[i - 1], times[i], times[i + 1]);
      }
      else if (i == times.size() - 1)
      {
        theta0_d = JointSpaceTrajectoryPlanner::via_velocity(thetas[i - 1], thetas[i], thetas[i + 1], times[i - 1], times[i], times[i + 1]);
        thetaf_d = 0.0;
      }
      else
      {
        theta0_d = JointSpaceTrajectoryPlanner::via_velocity(thetas[i - 2], thetas[i - 1], thetas[i], times[i - 2], times[i - 1], times[i]);
        thetaf_d = JointSpaceTrajectoryPlanner::via_velocity(thetas[i - 1], thetas[i], thetas[i + 1], times[i - 1], times[i], times[i + 1]);
      }

      return JointSpaceTrajectoryPlanner::poly3c(thetas[i - 1], thetas[i], theta0_d, thetaf_d, times[i - 1], times[i], t);
    }
}

// 5th order polynomial with non-zero initial and final joints velocities and accelerations
Eigen::Vector3d JointSpaceTrajectoryPlanner::poly5c(const double theta0, const double thetaf, const double theta0_d,
                                                    const double thetaf_d, const double theta0_dd,
                                                    const double thetaf_dd, const double t0, const double tf,
                                                    const double t)
{
  double a0 = theta0;
  double a1 = theta0_d;
  double a2 = theta0_dd * 0.5;
  double a3 = (20 * thetaf - 20 * theta0 - (8 * thetaf_d + 12 * theta0_d) * (tf - t0) -
               (3 * theta0_dd - thetaf_dd) * pow(tf - t0, 2)) /
              (2 * pow(tf - t0, 3));
  double a4 = (30 * theta0 - 30 * thetaf + (14 * thetaf_d + 16 * theta0_d) * (tf - t0) +
               (3 * theta0_dd - 2 * thetaf_dd) * pow(tf - t0, 2)) /
              (2 * pow(tf - t0, 4));
  double a5 =
      (12 * thetaf - 12 * theta0 - (6 * thetaf_d + 6 * theta0_d) * (tf - t0) - (theta0_dd - thetaf_dd) * pow(tf, 2)) /
      (2 * pow(tf - t0, 5));

  double theta =
      a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3) + a4 * pow(t - t0, 4) + a5 * pow(t - t0, 5);
  double theta_d = a1 + 2 * a2 * (t - t0) + 3 * a3 * pow(t - t0, 2) + 4 * a4 * pow(t - t0, 3) + 5 * a5 * pow(t - t0, 4);
  double theta_dd = 2 * a2 + 6 * a3 * (t - t0) + 12 * a4 * pow(t - t0, 2) + 20 * a5 * pow(t - t0, 3);

  return Eigen::Vector3d(theta, theta_d, theta_dd);
}

// Linear segment with parabolic blends
Eigen::Vector3d JointSpaceTrajectoryPlanner::lspb(const double theta0, const double thetaf, const double thetab_dd,
                                                  const double t0, const double tf, const double t)
{
  if (abs(thetab_dd) < (4 * abs(thetaf - theta0)) / pow(tf - t0, 2))
  {
    ROS_ERROR("Invalid acceleration! The value needs to be larger.");
    return Eigen::Vector3d(-1, -1, -1);
  }

  double theta, theta_d, theta_dd;
  double delta_tb = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(thetaf - theta0) / abs(thetab_dd));
  double tb = t0 + delta_tb;
  double thetab_d = thetab_dd * delta_tb;
  double thetab = theta0 + 0.5 * thetab_dd * pow(delta_tb, 2);
  double thetabf = thetab + thetab_d * ((tf - t0) - 2 * delta_tb);

  if (t >= t0 && t <= tb)
  {
    theta_dd = thetab_dd;
    theta_d = thetab_dd * (t - t0);
    theta = theta0 + 0.5 * theta_dd * pow(t - t0, 2);
  }
  else if (t >= (tf - delta_tb) && t <= tf + 0.0001)
  {
    theta_dd = -thetab_dd;
    theta_d = thetab_d + theta_dd * (t - (tf - delta_tb));
    theta = thetabf + thetab_d * (t - (tf - delta_tb)) + 0.5 * theta_dd * pow(t - (tf - delta_tb), 2);
  }
  else
  {
    theta_dd = 0;
    theta_d = thetab_d;
    theta = thetab + thetab_d * (t - tb);
  }
  // std::cout << theta0 << " " << thetaf << " " << thetab_dd << " " << t0 << " " << tf << " " << tb << " " << t
  //           << std::endl;
  // std::cout << theta << " " << theta_d << " " << theta_dd << std::endl;
  // std::cout << thetab << " " << thetabf << std::endl;

  return Eigen::Vector3d(theta, theta_d, theta_dd);
}

// Linear segment with parabolic blends and via points
Eigen::Vector3d JointSpaceTrajectoryPlanner::lspb_vias(const std::vector<double> thetas,
                                                       const std::vector<double> times, const double thetab_dd,
                                                       const double t)
{
  if (thetas.size() != times.size())
  {
    ROS_ERROR("The thetas vector should be the same size as times vector.");
    return Eigen::Vector3d(-1, -1, -1);
  }

  double theta0_d, thetaf_d;
  double theta_dd = thetab_dd;
  for (size_t i = 1; i < times.size(); i++)
    if (t >= times[i - 1] && t <= times[i])
    {
      if (i == 1)
      {
        if (thetas[i] - thetas[i - 1] < 0)
          theta_dd = -thetab_dd;
      }
      else if (i == times.size() - 1)
      {
        if (thetas[i - 1] - thetas[i] < 0)
          theta_dd = -thetab_dd;
      }
      else
      {
        theta0_d = (thetas[i] - thetas[i - 1]) / (times[i] - times[i - 1]);
        thetaf_d = (thetas[i + 1] - thetas[i]) / (times[i + 1] - times[i]);
        if (thetaf_d - theta0_d < 0)
          theta_dd = -thetab_dd;
      }

      return JointSpaceTrajectoryPlanner::lspb(thetas[i - 1], thetas[i], theta_dd, times[i - 1], times[i], t);
    }
}
}  // namespace trajectory_planner