// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file trajectory.cpp
 * \brief Defines Trajectory class for robot arm trajectory planning.
 */

#include <smalldrop_toolpath/trajectory.h>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Trajectory::Trajectory(const poses_t poses, const double length, const double duration)
 */
Trajectory::Trajectory(const poses_t poses, const double length, const double duration)
  : poses_(poses), length_(length), duration_(duration)
{
}

/**
 * \copybrief Trajectory::Trajectory(const poses_t poses, const path_actions_t actions, const double length, const
 * double duration)
 */
Trajectory::Trajectory(const poses_t poses, const path_actions_t actions, const double length, const double duration)
  : poses_(poses), actions_(actions), length_(length), duration_(duration)
{
}

/**
 * \copybrief Trajectory::length() const
 */
double Trajectory::length() const
{
  return length_;
}

/**
 * \copybrief Trajectory::poses() const
 */
poses_t Trajectory::poses() const
{
  return poses_;
}

/**
 * \copybrief Trajectory::duration() const
 */
double Trajectory::duration() const
{
  return duration_;
}

/**
 * \copybrief Trajectory::empty() const
 */
bool Trajectory::empty() const
{
  return poses_.empty();
}

/**
 * \copybrief Trajectory::actions() const
 */
path_actions_t Trajectory::actions() const
{
  return actions_;
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

}  // namespace smalldrop_toolpath

}  // namespace smalldrop