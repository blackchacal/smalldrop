// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file circle.h
 * \brief Declares Circle class for robot arm path planning.
 */

#ifndef _SMALLDROP_CIRCLE_PATH_H
#define _SMALLDROP_CIRCLE_PATH_H

#include <smalldrop_toolpath/path.h>

// Libraries
#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class Circle
 * \brief Circle path class.
 */
class Circle : public Path
{
public:
  /**
   * \fn Circle(const pose_t current_pose, const pose_t center, const double radius, const unsigned int n_points, const PATH_PLANE plane = PATH_PLANE::XY);
   * \brief Constructor.
   * 
   * \param current_pose Current robot pose.
   * \param center Circular path center pose/point.
   * \param radius Circular path radius.
   * \param n_points Number of points that make the circle. More points means a more defined circle.
   * \param plane The geometrical plane where the path will be followed.
   */
  Circle(const pose_t current_pose, const pose_t center, const double radius, const unsigned int n_points, const PATH_PLANE plane = PATH_PLANE::XY);

  ~Circle()
  {
  }
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_CIRCLE_PATH_H