// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file line.h
 * \brief Declares Line class for robot arm path planning.
 */

#ifndef _SMALLDROP_LINE_PATH_H
#define _SMALLDROP_LINE_PATH_H

#include <smalldrop_toolpath/path.h>

// Libraries
#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class Line
 * \brief Line path class.
 */
class Line : public Path
{
public:
  /**
   * \fn Line(const pose_t pose_i, const pose_t pose_f)
   * \brief Constructor.
   */
  Line(const pose_t pose_i, const pose_t pose_f);

  ~Line()
  {
  }
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_LINE_PATH_H