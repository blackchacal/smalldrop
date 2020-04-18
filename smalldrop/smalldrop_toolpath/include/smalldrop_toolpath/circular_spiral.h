// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file circular_spiral.h
 * \brief Declares CircularSpiral class for robot arm path planning.
 */

#ifndef _SMALLDROP_CIRCULAR_SPIRAL_PATH_H
#define _SMALLDROP_CIRCULAR_SPIRAL_PATH_H

#include <smalldrop_toolpath/path.h>

// Libraries
#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class CircularSpiral
 * \brief CircularSpiral path class.
 */
class CircularSpiral : public Path
{
public:
  /**
   * \fn CircularSpiral(const pose_t current_pose, const double eradius, const double iradius, const unsigned int loops, const unsigned int n_points, const PATH_PLANE plane = PATH_PLANE::XY);
   * \brief Constructor.
   * 
   * \param current_pose Current robot pose.
   * \param eradius Circular spiral path external radius.
   * \param iradius Circular spiral path internal radius.
   * \param loops Number of spiral loops.
   * \param n_points Number of points that make the circular spiral. More points means a more defined circular spiral.
   * \param plane The geometrical plane where the path will be followed.
   */
  CircularSpiral(const pose_t current_pose, const double eradius, const double iradius, const unsigned int loops, const unsigned int n_points, const PATH_PLANE plane = PATH_PLANE::XY);

  ~CircularSpiral()
  {
  }
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_CIRCULAR_SPIRAL_PATH_H