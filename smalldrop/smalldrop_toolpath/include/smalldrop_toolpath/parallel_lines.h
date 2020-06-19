// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file parallel_lines.h
 * \brief Declares ParallelLines class for robot arm wound filling printing path planning.
 */

#ifndef _SMALLDROP_TOOL_PATH_PARALLEL_LINES_H
#define _SMALLDROP_TOOL_PATH_PARALLEL_LINES_H

#include <smalldrop_toolpath/toolpath.h>

using namespace smalldrop::smalldrop_segmentation;

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class ParallelLines
 * \brief ParallelLines wound filling printing path class.
 */
class ParallelLines : public ToolPath
{
public:
  /**
   * Class constructor & methods
   *****************************************************************************************/

  /**
   * \fn ParallelLines(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis, const double pose_z,
   * const img_wsp_calibration_t calibration_data) 
   * \brief Constructor for a parallel lines wound filling toolpath.
   *
   * \param contour List of opencv points that form a wound contour.
   * \param offset Distance between grid lines.
   * \param axis The axis the is parallel to the grid lines.
   * \param pose_z Z axis coordinate for the robot path execution.
   * \param calibration_data Data for image-workspace calibration.
   */
  ParallelLines(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis, const double pose_z,
                const img_wsp_calibration_t calibration_data);

  /**
   * \fn ParallelLines(const points_t contour, const poses_t poses_contour_region, const Eigen::Matrix4d& transform,
   * const unsigned int offset, const IMAGE_AXIS axis, const img_wsp_calibration_t calibration_data) 
   * \brief Constructor for a parallel lines wound filling toolpath.
   *
   * \param contour List of opencv points that form a wound contour.
   * \param poses_contour_region List of poses points contained inside the wound contour.
   * \param transform Transformation matrix between camera frame and robot base frame.
   * \param offset Distance between grid lines.
   * \param axis The axis the is parallel to the grid lines.
   * \param calibration_data Data for image-workspace calibration.
   */
  ParallelLines(const points_t contour, const poses_t poses_contour_region, const Eigen::Matrix4d& transform,
                const unsigned int offset, const IMAGE_AXIS axis, const img_wsp_calibration_t calibration_data);

  ~ParallelLines(){};
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TOOL_PATH_PARALLEL_LINES_H