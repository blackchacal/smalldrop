// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file line.cpp
 * \brief Defines Line class for robot arm path planning.
 */

#include <smalldrop_toolpath/line.h>

#include <cmath>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Line::Line(const pose_t pose_i, const pose_t pose_f)
 */
Line::Line(const pose_t pose_i, const pose_t pose_f) : Path()
{
  double t = 0;
  while (t < 1.0001)
  {
    pose_t pose;
    pose.position.x = (1 - t) * pose_i.position.x + t * pose_f.position.x;
    pose.position.y = (1 - t) * pose_i.position.y + t * pose_f.position.y;
    pose.position.z = (1 - t) * pose_i.position.z + t * pose_f.position.z;
    Eigen::Quaterniond qres;
    Eigen::Quaterniond qi(pose_i.orientation.w, pose_i.orientation.x, pose_i.orientation.y, pose_i.orientation.z);
    Eigen::Quaterniond qf(pose_f.orientation.w, pose_f.orientation.x, pose_f.orientation.y, pose_f.orientation.z);
    qres = qi.slerp(t, qf);
    pose.orientation.x = qres.x();
    pose.orientation.y = qres.y();
    pose.orientation.z = qres.z();
    pose.orientation.w = qres.w();
    poses_.push_back(pose);
    t++;
  }

  // Calculate path length.
  calcLength();
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop