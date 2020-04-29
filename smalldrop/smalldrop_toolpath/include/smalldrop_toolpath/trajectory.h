// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file trajectory.h
 * \brief Declares Trajectory class for robot arm trajectory planning.
 */

#ifndef _SMALLDROP_TRAJECTORY_H
#define _SMALLDROP_TRAJECTORY_H

#include <smalldrop_toolpath/path.h>
#include <smalldrop_toolpath/toolpath.h>

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
 * \class Trajectory
 * \brief Class for robot arm trajectory planning.
 */
class Trajectory
{
public:
  /**
   * Class Constructor & Destructor
   *****************************************************************************************/

  /**
   * \fn Trajectory(const poses_t poses, const double length, const double duration)
   * \brief Constructor for trajectories originated from paths.
   */
  Trajectory(const poses_t poses, const double length, const double duration);

  /**
   * \fn Trajectory(const poses_t poses, const path_actions_t actions, const double length, const double duration)
   * \brief Constructor for trajectories originated from toolpaths.
   */
  Trajectory(const poses_t poses, const path_actions_t actions, const double length, const double duration);

  ~Trajectory()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn double length() const
   * \brief Calculates the path length.
   */
  double length() const;

  /**
   * \fn poses_t poses() const
   * \brief Returns the path points.
   */
  poses_t poses() const;

  /**
   * \fn double duration() const
   * \brief Returns the trajectory duration.
   */
  double duration() const;

  /**
   * \fn bool empty() const
   * \brief Checks if trajectory has poses. It should be empty when planning duration is 0, planning frequency is zero
   * or associated path length is zero.
   */
  bool empty() const;

  /**
   * \fn path_actions_t actions() const
   * \brief Returns the trajectory actions.
   */
  path_actions_t actions() const;

private:
  /**
   * Class members
   *****************************************************************************************/

  poses_t poses_;          /** \var List of trajectory poses. */
  path_actions_t actions_; /** \var Print actions for each path pose. */
  double length_;          /** \var Trajectory length. */
  double duration_;        /** \var Trajectory duration in seconds. */
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TRAJECTORY_H