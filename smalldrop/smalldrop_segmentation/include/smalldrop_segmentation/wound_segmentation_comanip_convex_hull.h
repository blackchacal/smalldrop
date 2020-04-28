// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_comanip_convex_hull.h
 * \brief Declares class for co-manipulation wound segmentation convex hull algorithm.
 */

#ifndef _SMALLDROP_WOUND_SEGMENTATION_COMANIP_CONVEX_HULL_H
#define _SMALLDROP_WOUND_SEGMENTATION_COMANIP_CONVEX_HULL_H

#include <smalldrop_segmentation/wound_segmentation_comanip.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/**
 * \class WSegmentCoManipConvexHull
 * \brief Class for co-manipulation wound segmentation convex hull algorithm.
 */
class WSegmentCoManipConvexHull : public WSegmentCoManip
{
public:
  /**
   * Class Constructors & Destructors
   *****************************************************************************************/

  /**
   * \fn WSegmentCoManipConvexHull(const std::string filepath, const img_wsp_calibration_t calibration_data) 
   * \brief Constructor where robot workspace and image limits are defined.
   *
   * \param filepath The path to the file with wound segmentation poses data.
   * \param calibration_data Data for image-workspace calibration.
   */
  WSegmentCoManipConvexHull(const std::string filepath, const img_wsp_calibration_t calibration_data);

  ~WSegmentCoManipConvexHull()
  {
  }
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_COMANIP_CONVEX_HULL_H