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
   * \fn WSegmentCoManipConvexHull(const std::string filepath, const unsigned int im_width, const unsigned int
   * im_height, const double wsp_x_min, const double wsp_x_max, const double wsp_y_min, const double wsp_y_max) 
   * \brief Constructor where robot workspace and image limits are defined.
   *
   * \param filepath The path to the file with wound segmentation poses data.
   * \param im_width Image width in pixels
   * \param im_height Image height in pixels
   * \param wsp_x_min Robot workspace coordinates minimum x limit
   * \param wsp_x_max Robot workspace coordinates maximum x limit
   * \param wsp_y_min Robot workspace coordinates minimum y limit
   * \param wsp_y_max Robot workspace coordinates maximum y limit
   */
  WSegmentCoManipConvexHull(const std::string filepath, const unsigned int im_width, const unsigned int im_height,
                            const double wsp_x_min, const double wsp_x_max, const double wsp_y_min,
                            const double wsp_y_max);

  ~WSegmentCoManipConvexHull()
  {
  }
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_COMANIP_CONVEX_HULL_H