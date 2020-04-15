// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file path.cpp
 * \brief Defines Path base class for robot arm path planning.
 */

#include <smalldrop_toolpath/path.h>

#include <cmath>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Path::length() const
 */
double Path::length() const
{
  return length_;
}

/**
 * \copybrief Path::points() const
 */
poses_t Path::poses() const
{
  return poses_;
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \copybrief Path::calcLength()
 */
void Path::calcLength()
{
  if (poses_.size() >= 2)
    for (size_t i = 0; i < poses_.size() - 1; i++)
      length_ += distanceBetweenTwoPoses(poses_[i], poses_[i+1]);
  else
    length_ = 0;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief Path::distanceBetweenTwoPoses(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f) const
 */
double Path::distanceBetweenTwoPoses(const geometry_msgs::Pose pose_i, const geometry_msgs::Pose pose_f) const
{
  return std::sqrt(std::pow((pose_f.position.x - pose_i.position.x), 2) +
                   std::pow((pose_f.position.y - pose_i.position.y), 2) +
                   std::pow((pose_f.position.z - pose_i.position.z), 2));
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop