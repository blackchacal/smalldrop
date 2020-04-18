// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file circle.cpp
 * \brief Defines Circle class for robot arm path planning.
 */

#include <smalldrop_toolpath/circle.h>

#include <cmath>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Circle::Circle(const pose_t current_pose, const pose_t center, const double
 * radius, const unsigned int n_points, const PATH_PLANE plane)
 */
Circle::Circle(const pose_t current_pose, const pose_t center, const double radius,
               const unsigned int n_points, const PATH_PLANE plane)
  : Path()
{
  double t = 0;
  double step = 1 / (double)n_points;
  while (t <= 1.0001)
  {
    pose_t pose;
    switch (plane)
    {
      case PATH_PLANE::XZ:
        pose.position.x = radius * cos(2 * M_PI * t) + center.position.x;
        pose.position.y = center.position.y;
        pose.position.z = radius * sin(2 * M_PI * t) + center.position.z;
        break;
      case PATH_PLANE::YZ:
        pose.position.x = center.position.x;
        pose.position.y = radius * cos(2 * M_PI * t) + center.position.y;
        pose.position.z = radius * sin(2 * M_PI * t) + center.position.z;
        break;
      default:
        pose.position.x = radius * cos(2 * M_PI * t) + center.position.x;
        pose.position.y = radius * sin(2 * M_PI * t) + center.position.y;
        pose.position.z = center.position.z;
        break;
    }
    pose.orientation.x = current_pose.orientation.x;
    pose.orientation.y = current_pose.orientation.y;
    pose.orientation.z = current_pose.orientation.z;
    pose.orientation.w = current_pose.orientation.w;

    poses_.push_back(pose);
    t += step;
  }

  // Calculate path length.
  calcLength();
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop