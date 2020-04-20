// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file trajectory_planner.h
 * \brief Declares TrajectoryPlanner class for robot arm trajectory planning.
 */

#ifndef _SMALLDROP_TRAJECTORY_PLANNER_H
#define _SMALLDROP_TRAJECTORY_PLANNER_H

#include <smalldrop_toolpath/path.h>
#include <smalldrop_toolpath/trajectory.h>

// ROS messages
#include <geometry_msgs/Pose.h>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \typedef pose_t
 */
typedef geometry_msgs::Pose pose_t;

/**
 * \typedef poses_t
 */
typedef std::vector<geometry_msgs::Pose> poses_t;

/**
 * \typedef paths_t
 */
typedef std::vector<Path> paths_t;

enum class PLAN_MODE
{
  POLY3,
  POLY3_VIAS,
  LSPB
};

/**
 * \class TrajectoryPlanner
 * \brief Class for robot arm trajectory planning.
 */
class TrajectoryPlanner
{
public:
  /**
   * Public constants
   *****************************************************************************************/
  const double LSPB_ACCEL = 30;

  /**
   * \fn TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode)
   * \brief Constructor.
   *
   * \param duration Trajectory duration.
   * \param frequency Frequency used to send trajectory poses to the robot.
   * \param plan_mode Trajectory planning mode.
   */
  TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode);

  /**
   * \fn TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode, const double max_speed) 
   * \brief Constructor.
   *
   * \param duration Trajectory duration.
   * \param frequency Frequency used to send trajectory poses to the robot.
   * \param plan_mode Trajectory planning mode.
   * \param max_speed Maximum allowed planning speed on cartesian space (mm/s). It will indirectly set boundaries on
   * trajectory duration.
   */
  TrajectoryPlanner(const double duration, const double frequency, const PLAN_MODE plan_mode, const double max_speed);

  ~TrajectoryPlanner()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn Trajectory plan(const Path& path)
   * \brief Plan the trajectory.
   *
   * \param path Path associated to the trajectory.
   */
  Trajectory plan(const Path& path);

  /**
   * \fn Trajectory plan(const paths_t paths)
   * \brief Plan the trajectory.
   *
   * \param paths A list of paths that will be concatenated to form the trajectory.
   */
  Trajectory plan(const paths_t paths);

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

  /**
   * \fn void setMaxSpeed(const double max_speed)
   * \brief Sets the maximum planning speed to generate the trajectory.
   *
   * \param max_speed Trajectory maximum planning speed.
   */
  void setMaxSpeed(const double max_speed);

private:
  /**
   * Class members
   *****************************************************************************************/

  double duration_;     /** \var trajectory duration. */
  double frequency_;    /** \var Trajectory sampling frequency. */
  PLAN_MODE plan_mode_; /** \var Planning mode used. */
  double max_speed_;    /** \var Maximum planning speed on cartesian space (mm/s). */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn double viaVelocity(const double pos_before, const double pos, const double pos_after, const double t_before,
   * const double t, const double t_after) const \brief Calculate via velocity based on heuristic.
   */
  double viaVelocity(const double pos_before, const double pos, const double pos_after, const double t_before,
                     const double t, const double t_after) const;

  /**
   * \fn pose_t poly3(const pose_t pose_i, const pose_t pose_f, const double t0, const double tf, const double t) const
   * \brief Generates intermediate poses, between two different poses, using 3rd order polynomial with zero initial and
   * final velocities.
   *
   * \param pose_i Initial pose.
   * \param pose_f Final pose.
   * \param t0 Initial time.
   * \param tf Final time.
   * \param t Intermediate time.
   */
  pose_t poly3(const pose_t pose_i, const pose_t pose_f, const double t0, const double tf, const double t) const;

  /**
   * \fn pose_t poly3c(const pose_t pose_i, const pose_t pose_f, const std::vector<double> velocity0, const
   * std::vector<double> velocityf, const double t0, const double tf, const double t) const \brief Generates
   * intermediate poses, between two different poses, using 3rd order polynomial with non-zero initial and final
   * velocities.
   *
   * \param pose_i Initial pose.
   * \param pose_f Final pose.
   * \param v0 Initial velocity
   * \param vf Final velocity
   * \param t0 Initial time.
   * \param tf Final time.
   * \param t Intermediate time.
   */
  pose_t poly3c(const pose_t pose_i, const pose_t pose_f, const std::vector<double> velocity0,
                const std::vector<double> velocityf, const double t0, const double tf, const double t) const;

  /**
   * \fn pose_t poly3cVias(const poses_t poses, const std::vector<double> times, const double t) const
   * \brief Generates intermediate poses, between two different poses, using 3rd order polynomial with via points and
   * velocity heuristics.
   *
   * \param poses Intermediate poses through which the trajectory should pass (via poses)
   * \param times List of times, initial, final and via poses.
   * \param t Intermediate time.
   */
  pose_t poly3cVias(const poses_t poses, const std::vector<double> times, const double t) const;

  /**
   * \fn pose_t lspb(const pose_t pose_i, const pose_t pose_f, const double accel, const double t0, const double tf,
   * const double t) const \brief Generates intermediate poses, between two different poses, using Linear segment with
   * parabolic blends.
   *
   * \param pose_i Initial pose.
   * \param pose_f Final pose.
   * \param accel Acceleration during parabolic blends.
   * \param t0 Initial time.
   * \param tf Final time.
   * \param t Intermediate time.
   */
  pose_t lspb(const pose_t pose_i, const pose_t pose_f, const double accel, const double t0, const double tf,
              const double t) const;

  /**
   * \fn double getFullPathLength(const paths_t path)
   * \brief Get the length of all the paths associated with the trajectory
   * 
   * \param paths List of paths.
   */
  double getFullPathLength(const paths_t paths);

  /**
   * \fn double getFullPathSize(const paths_t path)
   * \brief Get the size of all the paths associated with the trajectory
   * 
   * \param paths List of paths.
   */
  double getFullPathSize(const paths_t paths);
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TRAJECTORY_PLANNER_H