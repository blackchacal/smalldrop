// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file zigzag.h
 * \brief Declares ZigZag class for robot arm wound filling printing path planning.
 */

#ifndef _SMALLDROP_TOOL_PATH_ZIGZAG_H
#define _SMALLDROP_TOOL_PATH_ZIGZAG_H

#include <smalldrop_toolpath/toolpath.h>

#include <Eigen/Dense>

using namespace smalldrop::smalldrop_segmentation;

namespace smalldrop
{
namespace smalldrop_toolpath
{
/**
 * \class ZigZag
 * \brief ZigZag wound filling printing path class.
 */
class ZigZag : public ToolPath
{
public:
  /**
   * Class constructor & methods
   *****************************************************************************************/

  /**
   * \fn ZigZag(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis, const double pose_z, const
   * img_wsp_calibration_t calibration_data)
   * \brief Constructor for a zig zag wound filling toolpath.
   *
   * \param contour List of opencv points that form a wound contour.
   * \param offset Distance between grid lines.
   * \param axis The axis the is parallel to the grid lines.
   * \param pose_z Z axis coordinate for the robot path execution.
   * \param calibration_data Data for image-workspace calibration.
   */
  ZigZag(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis, const double pose_z,
         const img_wsp_calibration_t calibration_data);

  /**
   * \fn ZigZag(const points_t contour, const poses_t poses_contour_region, const Eigen::Matrix4d& transform, const
   * unsigned int offset, const IMAGE_AXIS axis, const double pose_z, const img_wsp_calibration_t calibration_data)
   * \brief Constructor for a zig zag wound filling toolpath.
   *
   * \param contour List of opencv points that form a wound contour.
   * \param poses_contour_region List of poses points contained inside the wound contour
   * \param transform Transformation matrix between camera frame and robot base frame.
   * \param offset Distance between grid lines.
   * \param axis The axis the is parallel to the grid lines.
   * \param calibration_data Data for image-workspace calibration.
   */
  ZigZag(const points_t contour, const poses_t poses_contour_region, const Eigen::Matrix4d& transform,
         const unsigned int offset, const IMAGE_AXIS axis, const img_wsp_calibration_t calibration_data);

  ~ZigZag(){};
};

}  // namespace smalldrop_toolpath

}  // namespace smalldrop

#endif  // _SMALLDROP_TOOL_PATH_ZIGZAG_H