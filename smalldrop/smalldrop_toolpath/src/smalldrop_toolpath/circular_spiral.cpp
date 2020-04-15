// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file circular_spiral.cpp
 * \brief Defines CircularSpiral class for robot arm path planning.
 */

#include <smalldrop_toolpath/circular_spiral.h>

#include <cmath>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief CircularSpiral::CircularSpiral(const geometry_msgs::Pose current_pose, const double eradius, const double
 * iradius, const unsigned int loops, const unsigned int n_points, const PATH_PLANE plane)
 */
CircularSpiral::CircularSpiral(const geometry_msgs::Pose current_pose, const double eradius, const double iradius,
                               const unsigned int loops, const unsigned int n_points, const PATH_PLANE plane)
  : Path()
{
  double t;
  double step = 1 / (double)n_points;
  double rstep = (eradius - iradius) / (double)loops;
  double sub_rstep = rstep / (double)n_points;
  double radius = eradius;
  double i = eradius;

  while (i > iradius + 0.0001)
  {
    t = 0;
    while (t <= 0.9999)
    {
      geometry_msgs::Pose pose;
      switch (plane)
      {
      case PATH_PLANE::XZ:
        pose.position.x = radius * cos(2 * M_PI * t) + current_pose.position.x - eradius;
        pose.position.y = current_pose.position.y;
        pose.position.z = radius * sin(2 * M_PI * t) + current_pose.position.z;
        break;
      case PATH_PLANE::YZ:
        pose.position.x = current_pose.position.x;
        pose.position.y = radius * cos(2 * M_PI * t) + current_pose.position.y - eradius;
        pose.position.z = radius * sin(2 * M_PI * t) + current_pose.position.z;
        break;
      default:
        pose.position.x = radius * cos(2 * M_PI * t) + current_pose.position.x - eradius;
        pose.position.y = radius * sin(2 * M_PI * t) + current_pose.position.y;
        pose.position.z = current_pose.position.z;
        break;
      }
      pose.orientation.x = current_pose.orientation.x;
      pose.orientation.y = current_pose.orientation.y;
      pose.orientation.z = current_pose.orientation.z;
      pose.orientation.w = current_pose.orientation.w;

      poses_.push_back(pose);
      t += step;
      radius -= sub_rstep;
    }
    i -= rstep;
  }

  // Calculate path length.
  calcLength();
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop