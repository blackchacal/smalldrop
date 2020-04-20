// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file joint_trajectory_planner.h
 * \brief Declares JointTrajectoryPlanner class for robot arm joint trajectory planning.
 */

#ifndef _SMALLDROP_JOINT_TRAJECTORY_PLANNER_H
#define _SMALLDROP_JOINT_TRAJECTORY_PLANNER_H

#include <vector>

namespace smalldrop
{
namespace smalldrop_toolpath
{

/**
 * \typedef jpos_t
 * \brief Vector of joint positions.
 */
typedef std::vector<double> jpos_t;

enum class PLAN_MODE
{
  POLY3,
  LSPB
};

/**
 * \class JointTrajectoryPlanner
 * \brief Class for robot arm joint trajectory planning.
 */
class JointTrajectoryPlanner
{
public:
  /**
   * Public constants
   *****************************************************************************************/
  const double LSPB_ACCEL = 30;

  /**
   * \fn JointTrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode)
   * \brief Constructor.
   *
   * \param duration Trajectory duration.
   * \param frequency Frequency used to send trajectory poses to the robot.
   * \param plan_mode Trajectory planning mode.
   */
  JointTrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode);

  ~JointTrajectoryPlanner()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn std::vector<jpos_t> plan(const jpos_t joints_i, const jpos_t joints_f)
   * \brief Plan the joint trajectory.
   *
   * \param joints_i Initial joint configuration.
   * \param joints_f Final joint configuration.
   */
  std::vector<jpos_t> plan(const jpos_t joints_i, const jpos_t joints_f);

  /**
   * \fn void setDuration(const double duration)
   * \brief Sets the duration for the planned trajectory.
   *
   * \param duration Planned trajectory duration.
   */
  void setDuration(const double duration);

  /**
   * \fn void setFrequency(const double frequency)
   * \brief Sets the frequency for the planned trajectory.
   *
   * \param frequency Planned trajectory frequency.
   */
  void setFrequency(const double frequency);

  /**
   * \fn void setPlanMode(const PLAN_MODE plan_mode)
   * \brief Sets the planning mode to generate the trajectory.
   *
   * \param plan_mode Trajectory planning mode.
   */
  void setPlanMode(const PLAN_MODE plan_mode);

private:
  /**
   * Class members
   *****************************************************************************************/

  double duration_;     /** \var trajectory duration. */
  double frequency_;    /** \var Trajectory sampling frequency. */
  PLAN_MODE plan_mode_; /** \var Planning mode used. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn double poly3(const double theta_i, const double theta_f, const double t0, const double tf, const double t) const
   * \brief Generates intermediate joint positions, between two different positions, using 3rd order polynomial with zero initial and
   * final velocities.
   *
   * \param theta_i Initial theta.
   * \param theta_f Final theta.
   * \param t0 Initial time.
   * \param tf Final time.
   * \param t Intermediate time.
   */
  double poly3(const double theta_i, const double theta_f, const double t0, const double tf, const double t) const;

  /**
   * \fn double lspb(const double theta_i, const double theta_f, const double accel, const double t0, const double tf,
   * const double t) const 
   * \brief Generates intermediate poses, between two different poses, using Linear segment with
   * parabolic blends.
   *
   * \param theta_i Initial theta.
   * \param theta_f Final theta.
   * \param accel Acceleration during parabolic blends.
   * \param t0 Initial time.
   * \param tf Final time.
   * \param t Intermediate time.
   */
  double lspb(const double theta_i, const double theta_f, const double accel, const double t0, const double tf,
              const double t) const;
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_JOINT_TRAJECTORY_PLANNER_H