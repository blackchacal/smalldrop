// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file trajectory_planner.cpp
 * \brief Defines TrajectoryPlanner class for robot arm trajectory planning.
 */

#include <smalldrop_toolpath/trajectory_planner.h>
#include <smalldrop_state/exceptions.h>

#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief TrajectoryPlanner::TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE
 * plan_mode)
 */
TrajectoryPlanner::TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode)
  : duration_(duration), frequency_(frequency), plan_mode_(plan_mode), max_speed_(50)
{
}

/**
 * \copybrief TrajectoryPlanner::TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode, const double max_speed) 
 */
TrajectoryPlanner::TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode, const double max_speed)
  : duration_(duration), frequency_(frequency), plan_mode_(plan_mode), max_speed_(max_speed)
{
}

/**
 * \copybrief TrajectoryPlanner::plan(const Path& path)
 */
Trajectory TrajectoryPlanner::plan(const Path& path)
{
  // Check if speed exceeds upper bound
  if (duration_ > 0 && path.length()/duration_ > max_speed_)
    throw smalldrop_state::TrajectoryMaxSpeedExceededException();

  poses_t trajectory;
  poses_t poses = path.poses();
  unsigned int npoints = poses.size();
  double t_interval = duration_ / ((double)npoints-1);

  Trajectory t(trajectory, 0, duration_);

  if (duration_ > 0 && frequency_ > 0)
  {
    for (size_t i = 0; i < npoints - 1; i++)
    {
      double t = 0;
      while (t <= t_interval + 0.0001)
      {
        switch (plan_mode_)
        {
          case PLAN_MODE::LSPB:
            trajectory.push_back(lspb(poses[i], poses[i + 1], LSPB_ACCEL, 0, t_interval, t));
            break;
          default:  // PLAN_MODE::POLY3
            trajectory.push_back(poly3(poses[i], poses[i + 1], 0, t_interval, t));
            break;
        }
        t += 1 / frequency_;
      }
    }

    Trajectory traj(trajectory, path.length(), duration_);
    t = traj;
  }

  return t;
}

/**
 * \copybrief TrajectoryPlanner::plan(const paths_t paths)
 */
Trajectory TrajectoryPlanner::plan(const paths_t paths)
{
  // Check if speed exceeds upper bound
  if (duration_ > 0 && getFullPathLength(paths)/duration_ > max_speed_)
    throw smalldrop_state::TrajectoryMaxSpeedExceededException();

  poses_t trajectory;
  double path_duration = duration_ / paths.size();

  Trajectory t(trajectory, 0, duration_);

  if (duration_ > 0 && frequency_ > 0)
  {
    for (size_t i = 0; i < paths.size(); i++)
    {
      poses_t poses = paths[i].poses();
      unsigned int npoints = poses.size();
      double t_interval = path_duration / ((double)npoints-1);

      for (size_t j = 0; j < npoints - 1; j++)
      {
        double t = 0;
        while (t <= t_interval + 0.0001)
        {
          switch (plan_mode_)
          {
            case PLAN_MODE::LSPB:
              trajectory.push_back(lspb(poses[j], poses[j + 1], LSPB_ACCEL, 0, t_interval, t));
              break;
            default:  // PLAN_MODE::POLY3
              trajectory.push_back(poly3(poses[j], poses[j + 1], 0, t_interval, t));
              break;
          }
          t += 1 / frequency_;
        }
      }
    }

    Trajectory traj(trajectory, getFullPathLength(paths), duration_);
    t = traj;
  }

  return t;
}

/**
 * \copybrief TrajectoryPlanner::setDuration()
 */
void TrajectoryPlanner::setDuration(const double duration)
{
  duration_ = duration;
}

/**
 * \copybrief TrajectoryPlanner::setFrequency()
 */
void TrajectoryPlanner::setFrequency(const double frequency)
{
  frequency_ = frequency;
}

/**
 * \copybrief TrajectoryPlanner::setPlanMode()
 */
void TrajectoryPlanner::setPlanMode(const PLAN_MODE plan_mode)
{
  plan_mode_ = plan_mode;
}

/**
 * \copybrief TrajectoryPlanner::setMaxSpeed(const double max_speed)
 */
void TrajectoryPlanner::setMaxSpeed(const double max_speed)
{
  max_speed_ = max_speed;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief double viaVelocity(const double pos_before, const double pos, const double pos_after, const double
 * t_before, const double t, const double t_after) const
 */
double TrajectoryPlanner::viaVelocity(const double pos_before, const double pos, const double pos_after,
                                      const double t_before, const double t, const double t_after) const
{
  double velocity_i, velocity_ii;

  velocity_i = (pos - pos_before) / (t - t_before);  // Mean velocity before via point
  velocity_ii = (pos_after - pos) / (t_after - t);   // Mean velocity after via point

  if ((velocity_i * velocity_ii) < 0)  // Mean velocities change signal at via point
    return 0.0;                        // Velocity at via point is zero
  else
    return 0.5 * (velocity_ii + velocity_i);  // Velocity at via point is the average of the mean velocities
}

/**
 * \copybrief pose_t poly3(const pose_t pose_i, const pose_t pose_f, const double t0, const double tf, const double t)
 * const
 */
pose_t TrajectoryPlanner::poly3(const pose_t pose_i, const pose_t pose_f, const double t0, const double tf,
                                const double t) const
{
  std::vector<double> a0 = { pose_i.position.x, pose_i.position.y, pose_i.position.z };
  std::vector<double> a1 = { 0, 0, 0 };
  std::vector<double> a2 = { (3 / pow(tf - t0, 2)) * (pose_f.position.x - pose_i.position.x),
                             (3 / pow(tf - t0, 2)) * (pose_f.position.y - pose_i.position.y),
                             (3 / pow(tf - t0, 2)) * (pose_f.position.z - pose_i.position.z) };
  std::vector<double> a3 = { -(2 / pow(tf - t0, 3)) * (pose_f.position.x - pose_i.position.x),
                             -(2 / pow(tf - t0, 3)) * (pose_f.position.y - pose_i.position.y),
                             -(2 / pow(tf - t0, 3)) * (pose_f.position.z - pose_i.position.z) };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose_i.orientation.w, pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z);
  Eigen::Quaterniond qf(pose_f.orientation.w, pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z);
  qres = q0.slerp((t - t0) / (tf - t0), qf);

  pose_t pose;
  pose.position.x = a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3);
  pose.position.y = a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3);
  pose.position.z = a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  return pose;
}

/**
 * \copybrief pose_t poly3c(const pose_t pose_i, const pose_t pose_f, const std::vector<double> velocity0, const
 * std::vector<double> velocityf, const double t0, const double tf, const double t) const
 */
pose_t TrajectoryPlanner::poly3c(const pose_t pose_i, const pose_t pose_f, const std::vector<double> velocity0,
                                 const std::vector<double> velocityf, const double t0, const double tf,
                                 const double t) const
{
  std::vector<double> a0 = { pose_i.position.x, pose_i.position.y, pose_i.position.z };
  std::vector<double> a1 = { velocity0[0], velocity0[1], velocity0[2] };
  std::vector<double> a2 = { (3 / pow(tf - t0, 2)) * (pose_f.position.x - pose_i.position.x) -
                                 (2 / (tf - t0)) * velocity0[0] - (1 / (tf - t0)) * velocityf[0],
                             (3 / pow(tf - t0, 2)) * (pose_f.position.y - pose_i.position.y) -
                                 (2 / (tf - t0)) * velocity0[1] - (1 / (tf - t0)) * velocityf[1],
                             (3 / pow(tf - t0, 2)) * (pose_f.position.z - pose_i.position.z) -
                                 (2 / (tf - t0)) * velocity0[2] - (1 / (tf - t0)) * velocityf[2] };
  std::vector<double> a3 = { -(2 / pow(tf - t0, 3)) * (pose_f.position.x - pose_i.position.x) +
                                 (1 / pow(tf - t0, 2)) * (velocityf[0] + velocity0[0]),
                             -(2 / pow(tf - t0, 3)) * (pose_f.position.y - pose_i.position.y) +
                                 (1 / pow(tf - t0, 2)) * (velocityf[1] + velocity0[1]),
                             -(2 / pow(tf - t0, 3)) * (pose_f.position.z - pose_i.position.z) +
                                 (1 / pow(tf - t0, 2)) * (velocityf[2] + velocity0[2]) };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose_i.orientation.w, pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z);
  Eigen::Quaterniond qf(pose_f.orientation.w, pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z);
  qres = q0.slerp((t - t0) / (tf - t0), qf);

  pose_t pose;
  pose.position.x = a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3);
  pose.position.y = a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3);
  pose.position.z = a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  return pose;
}

/**
 * \copybrief pose_t poly3cVias(const poses_t poses, const std::vector<double> times, const double t) const
 */
pose_t TrajectoryPlanner::poly3cVias(const poses_t poses, const std::vector<double> times, const double t) const
{
  if (poses.size() != times.size())
  {
    throw std::runtime_error("The poses vector should be the same size as times vector.");
  }

  std::vector<double> velocity0;
  std::vector<double> velocityf;
  velocity0.resize(3, 0);
  velocityf.resize(3, 0);

  pose_t pose;
  for (size_t i = 1; i < times.size(); i++)
  {
    if (t >= times[i - 1] && t <= times[i])
    {
      if (i == 1)
      {
        // velocity0 is already {0, 0, 0}
        velocityf[0] = viaVelocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1],
                                   times[i], times[i + 1]);
        velocityf[1] = viaVelocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1],
                                   times[i], times[i + 1]);
        velocityf[2] = viaVelocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1],
                                   times[i], times[i + 1]);
      }
      else if (i == times.size() - 1)
      {
        velocity0[0] = viaVelocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1],
                                   times[i], times[i + 1]);
        velocity0[1] = viaVelocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1],
                                   times[i], times[i + 1]);
        velocity0[2] = viaVelocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1],
                                   times[i], times[i + 1]);
        // velocityf is already {0, 0, 0}
      }
      else
      {
        velocity0[0] = viaVelocity(poses[i - 2].position.x, poses[i - 1].position.x, poses[i].position.x, times[i - 1],
                                   times[i], times[i + 1]);
        velocity0[1] = viaVelocity(poses[i - 2].position.y, poses[i - 1].position.y, poses[i].position.y, times[i - 1],
                                   times[i], times[i + 1]);
        velocity0[2] = viaVelocity(poses[i - 2].position.z, poses[i - 1].position.z, poses[i].position.z, times[i - 1],
                                   times[i], times[i + 1]);

        velocityf[0] = viaVelocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1],
                                   times[i], times[i + 1]);
        velocityf[1] = viaVelocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1],
                                   times[i], times[i + 1]);
        velocityf[2] = viaVelocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1],
                                   times[i], times[i + 1]);
      }

      pose = poly3c(poses[i - 1], poses[i], velocity0, velocityf, times[i - 1], times[i], t);
      break;
    }
  }

  return pose;
}

/**
 * \copybrief pose_t lspb(const pose_t pose_i, const pose_t pose_f, const double accel, const double t0, const double
 * tf, const double t) const
 */
pose_t TrajectoryPlanner::lspb(const pose_t pose_i, const pose_t pose_f, const double accel, const double t0,
                               const double tf, const double t) const
{
  // It is assumed that accel param is always positive

  if (accel < (4 * abs(pose_f.position.x - pose_i.position.x)) / pow(tf - t0, 2) ||
      accel < (4 * abs(pose_f.position.y - pose_i.position.y)) / pow(tf - t0, 2) ||
      accel < (4 * abs(pose_f.position.z - pose_i.position.z)) / pow(tf - t0, 2))
  {
    throw std::runtime_error("Invalid acceleration! The value needs to be larger.");
  }

  // Change accel sign depending on growth direction of position (eg. x0 > xf => accel0 < 0)
  std::vector<double> accel_vec;
  accel_vec.resize(3, 0);
  accel_vec[0] = (pose_i.position.x > pose_f.position.x) ? -accel : accel;
  accel_vec[1] = (pose_i.position.y > pose_f.position.y) ? -accel : accel;
  accel_vec[2] = (pose_i.position.z > pose_f.position.z) ? -accel : accel;

  pose_t pose;
  std::vector<double> velocity, acceleration;
  velocity.resize(3, 0);
  acceleration.resize(3, 0);
  std::vector<double> delta_tb;
  delta_tb.resize(3, 0);
  delta_tb[0] =
      (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(pose_f.position.x - pose_i.position.x) / abs(accel_vec[0]));
  delta_tb[1] =
      (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(pose_f.position.y - pose_i.position.y) / abs(accel_vec[1]));
  delta_tb[2] =
      (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(pose_f.position.z - pose_i.position.z) / abs(accel_vec[2]));
  std::vector<double> tb;
  tb.resize(3, 0);
  tb[0] = t0 + delta_tb[0];
  tb[1] = t0 + delta_tb[1];
  tb[2] = t0 + delta_tb[2];
  std::vector<double> velocity_b;
  velocity_b.resize(3, 0);
  velocity_b[0] = accel_vec[0] * delta_tb[0];
  velocity_b[1] = accel_vec[1] * delta_tb[1];
  velocity_b[2] = accel_vec[2] * delta_tb[2];
  std::vector<double> poseb;
  poseb.resize(3, 0);
  poseb[0] = pose_i.position.x + 0.5 * accel_vec[0] * pow(delta_tb[0], 2);
  poseb[1] = pose_i.position.y + 0.5 * accel_vec[1] * pow(delta_tb[1], 2);
  poseb[2] = pose_i.position.z + 0.5 * accel_vec[2] * pow(delta_tb[2], 2);
  std::vector<double> posebf;
  posebf.resize(3, 0);
  posebf[0] = poseb[0] + velocity_b[0] * ((tf - t0) - 2 * delta_tb[0]);
  posebf[1] = poseb[1] + velocity_b[1] * ((tf - t0) - 2 * delta_tb[1]);
  posebf[2] = poseb[2] + velocity_b[2] * ((tf - t0) - 2 * delta_tb[2]);

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose_i.orientation.w, pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z);
  Eigen::Quaterniond qf(pose_f.orientation.w, pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z);
  qres = q0.slerp((t - t0) / (tf - t0), qf);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  for (size_t i = 0; i < 3; i++)
  {
    if (t >= t0 && t <= tb[i])
    {
      acceleration[i] = accel_vec[i];
      velocity[i] = accel_vec[i] * (t - t0);
      switch (i)
      {
        case 1:
          pose.position.y = pose_i.position.y + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
        case 2:
          pose.position.z = pose_i.position.z + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
        default:
          pose.position.x = pose_i.position.x + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
      }
    }
    else if (t >= (tf - delta_tb[i]) && t <= tf + 0.0001)
    {
      acceleration[i] = -accel_vec[i];
      velocity[i] = velocity_b[i] + acceleration[i] * (t - (tf - delta_tb[i]));
      switch (i)
      {
        case 1:
          pose.position.y = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) +
                            0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
        case 2:
          pose.position.z = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) +
                            0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
        default:
          pose.position.x = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) +
                            0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
      }
    }
    else
    {
      acceleration[i] = 0;
      velocity[i] = velocity_b[i];
      switch (i)
      {
        case 1:
          pose.position.y = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
        case 2:
          pose.position.z = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
        default:
          pose.position.x = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
      }
    }
  }

  return pose;
}

/**
 * \copybrief TrajectoryPlanner::getFullPathLength(const paths_t path)
 */
double TrajectoryPlanner::getFullPathLength(const paths_t paths)
{
  double total_length = 0;
  for (size_t i = 0; i < paths.size(); i++)
    total_length += paths[i].length();
  
  return total_length;
}

/**
 * \copybrief TrajectoryPlanner::getFullPathSize(const paths_t path)
 */
double TrajectoryPlanner::getFullPathSize(const paths_t paths)
{
  double total_size = 0;
  for (size_t i = 0; i < paths.size(); i++)
    total_size += paths[i].poses().size();
  
  return total_size;
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop