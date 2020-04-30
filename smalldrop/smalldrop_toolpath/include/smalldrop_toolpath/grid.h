// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file grid.h
 * \brief Declares Grid class for robot arm wound filling printing path planning.
 */

#ifndef _SMALLDROP_TOOL_PATH_GRID_H
#define _SMALLDROP_TOOL_PATH_GRID_H

#include <smalldrop_toolpath/toolpath.h>

using namespace smalldrop::smalldrop_segmentation;

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class Grid
 * \brief Grid wound filling printing path class.
 */
class Grid : public ToolPath
{
public:
  /**
   * Class constructor & methods
   *****************************************************************************************/

  /**
   * \fn Grid(const points_t contour, const unsigned int offset_x, const unsigned int offset_y,
   * const double pose_z, const img_wsp_calibration_t calibration_data)
   * \brief Constructor for a zig zag wound filling toolpath.
   *
   * \param contour List of opencv points that form a wound contour.
   * \param offset_x Distance between grid lines parallel to x axis.
   * \param offset_y Distance between grid lines parallel to y axis.
   * \param pose_z Z axis coordinate for the robot path execution.
   * \param calibration_data Data for image-workspace calibration.
   */
  Grid(const points_t contour, const unsigned int offset_x, const unsigned int offset_y, const double pose_z,
       const img_wsp_calibration_t calibration_data);

  ~Grid(){};
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TOOL_PATH_GRID_H