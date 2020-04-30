// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file grid.cpp
 * \brief Defines Grid class for robot arm wound filling printing path planning.
 */

#include <smalldrop_toolpath/grid.h>
#include <smalldrop_toolpath/parallel_lines.h>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Grid::Grid(const points_t contour, const unsigned int offset_x, const unsigned int offset_y, const double
 * pose_z, const img_wsp_calibration_t calibration_data)
 */
Grid::Grid(const points_t contour, const unsigned int offset_x, const unsigned int offset_y, const double pose_z,
           const img_wsp_calibration_t calibration_data)
  : ToolPath(calibration_data)
{
  points_t path_x, path_y;
  ParallelLines pl_x(contour, offset_x, IMAGE_AXIS::X, pose_z, calibration_data);
  ParallelLines pl_y(contour, offset_y, IMAGE_AXIS::Y, pose_z, calibration_data);
  path_x = pl_x.points();
  path_y = pl_y.points();
  std::reverse(std::begin(path_y), std::end(path_y));

  // Create new vector by concatenation
  points_.reserve(path_x.size() + path_y.size());
  points_.insert(points_.end(), path_x.begin(), path_x.end());
  points_.insert(points_.end(), path_y.begin(), path_y.end());

  poses_ = convPathPointToPose(points_, pose_z);

  // Add printing actions to path points
  for (unsigned int i = 0; i < poses_.size(); i++)
  {
    if (i == 0)
      actions_.push_back(PRINT_ACTION::START);  // First pose
    else if (i == poses_.size() - 1)
      actions_.push_back(PRINT_ACTION::STOP);  // Last pose
    else if (i == path_x.size() - 1)
      actions_.push_back(PRINT_ACTION::STOP);  // Last pose of path_x
    else if (i == path_x.size())
      actions_.push_back(PRINT_ACTION::START);  // First pose of path_y
    else
      actions_.push_back(PRINT_ACTION::CONTINUE);  // Last pose
  }

  // Calculate path length.
  calcLength();
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop