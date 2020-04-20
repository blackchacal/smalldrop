// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file joint_trajectory_planner.cpp
 * \brief Defines JointTrajectoryPlanner class for robot arm joint trajectory planning.
 */

#include <smalldrop_state/exceptions.h>
#include <smalldrop_toolpath/joint_trajectory_planner.h>

#include <cmath>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief JointTrajectoryPlanner::JointTrajectoryPlanner(const double duration, const double frequency, const
 * PLAN_MODE plan_mode)
 */
JointTrajectoryPlanner::JointTrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode)
  : duration_(duration), frequency_(frequency), plan_mode_(plan_mode)
{
}

/**
 * \copybrief JointTrajectoryPlanner::plan()
 */
std::vector<jpos_t> JointTrajectoryPlanner::plan(const jpos_t joints_i, const jpos_t joints_f)
{
  if (joints_i.size() != joints_f.size())
    throw smalldrop_state::InvalidNumberOfJointsException();

  std::vector<jpos_t> jpos_vec;
  double t = 0;
  while (t <= duration_ + 0.0001)
  {
    jpos_t jpos;
    for (size_t i = 0; i < joints_i.size(); i++)
    {
      switch (plan_mode_)
      {
        case PLAN_MODE::LSPB:
            jpos.push_back(lspb(joints_i[i], joints_f[i], LSPB_ACCEL, 0, duration_, t));
          break;
        default:  // PLAN_MODE::POLY3
            jpos.push_back(poly3(joints_i[i], joints_f[i], 0, duration_, t));
          break;
      }
    }
    jpos_vec.push_back(jpos);
    t += 1 / frequency_;
  }

  return jpos_vec;
}

/**
 * \copybrief JointTrajectoryPlanner::setDuration()
 */
void JointTrajectoryPlanner::setDuration(const double duration)
{
  duration_ = duration;
}

/**
 * \copybrief JointTrajectoryPlanner::setFrequency()
 */
void JointTrajectoryPlanner::setFrequency(const double frequency)
{
  frequency_ = frequency;
}

/**
 * \copybrief JointTrajectoryPlanner::setPlanMode()
 */
void JointTrajectoryPlanner::setPlanMode(const PLAN_MODE plan_mode)
{
  plan_mode_ = plan_mode;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief double poly3() const
 */
double JointTrajectoryPlanner::poly3(const double theta_i, const double theta_f, const double t0, const double tf,
                                     const double t) const
{
  double a0 = theta_i;
  double a1 = 0;
  double a2 = (3 / pow(tf - t0, 2)) * (theta_f - theta_i);
  double a3 = -(2 / pow(tf - t0, 3)) * (theta_f - theta_i);

  double theta = a0 + a1 * (t - t0) + a2 * pow(t - t0, 2) + a3 * pow(t - t0, 3);

  return theta;
}

/**
 * \copybrief double lspb() const
 */
double JointTrajectoryPlanner::lspb(const double theta_i, const double theta_f, const double accel, const double t0,
                                    const double tf, const double t) const
{
  if (abs(accel) < (4 * abs(theta_f - theta_i)) / pow(tf - t0, 2))
    throw smalldrop_state::JointTrajectoryLSPBInvalidAccelerationException();

  double theta;
  double delta_tb = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(theta_f - theta_i) / abs(accel));
  double tb = t0 + delta_tb;
  double thetab_d = accel * delta_tb;
  double thetab = theta_i + 0.5 * accel * pow(delta_tb, 2);
  double thetabf = thetab + thetab_d * ((tf - t0) - 2 * delta_tb);

  if (t >= t0 && t <= tb)
    theta = theta_i + 0.5 * accel * pow(t - t0, 2);
  else if (t >= (tf - delta_tb) && t <= tf + 0.0001)
    theta = thetabf + thetab_d * (t - (tf - delta_tb)) + 0.5 * (-accel) * pow(t - (tf - delta_tb), 2);
  else
    theta = thetab + thetab_d * (t - tb);

  return theta;
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop