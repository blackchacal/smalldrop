// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file path.h
 * \brief Declares Path base class for robot arm path planning.
 */

#ifndef _SMALLDROP_PATH_H
#define _SMALLDROP_PATH_H

#include <vector>

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

enum class PATH_PLANE
{
  XY,
  XZ,
  YZ
};

/**
 * \class Path
 * \brief Base class for robot arm path planning.
 */
class Path
{
public:
  virtual ~Path()
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

protected:
  /**
   * Class members
   *****************************************************************************************/

  poses_t poses_; /** \var List of path poses. */

  double length_; /** \var Path length. */

  /**
   * Class methods & Constructor
   *****************************************************************************************/

  /**
   * \fn Path()
   * \brief Constructor. Cannot be instantiated directly.
   */
  Path() : length_(0) {}

  /**
   * \fn void calcLength()
   * \brief Calculate path length based on path poses.
   */
  void calcLength();

private:

  /**
   * \fn double distanceBetweenTwoPoses(const pose_t pose_i, const pose_t pose_f) const
   * \brief Calculates the distance in space between two pose positions.
   */
  double distanceBetweenTwoPoses(const pose_t pose_i, const pose_t pose_f) const;
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_PATH_H