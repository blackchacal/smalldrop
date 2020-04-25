// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_comanip_convex_hull.cpp
 * \brief Defines class for co-manipulation wound segmentation convex hull algorithm.
 */

#include <smalldrop_segmentation/wound_segmentation_comanip_convex_hull.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCoManipConvexHull::WSegmentCoManipConvexHull(const std::string filepath, const unsigned int
 * im_width, const unsigned int im_height, const double wsp_x_min, const double wsp_x_max, const double wsp_y_min, const
 * double wsp_y_max)
 */
WSegmentCoManipConvexHull::WSegmentCoManipConvexHull(const std::string filepath, const unsigned int im_width,
                                                     const unsigned int im_height, const double wsp_x_min,
                                                     const double wsp_x_max, const double wsp_y_min,
                                                     const double wsp_y_max)
  : WSegmentCoManip(filepath, im_width, im_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max)
{
  // Get save poses and convert them to points
  poses_t poses = loadWoundSegmentationPoses();
  points_t points;
  for(size_t i = 0; i < poses.size(); i++)
  {
    point_t pt = convPoseToPoint(poses[i]);
    points.push_back(pt);
  }

  // Use convex hull algorithm to get the contour from the given points/poses
  poses_t hull_poses;
  points_t hull_points;
  if (points.size() > 0) 
  {
    std::vector<int> hull_positions;
    cv::convexHull(cv::Mat(points), hull_positions, true);
    cv::convexHull(cv::Mat(points), hull_points, true);

    for (size_t i = 0; i < hull_positions.size(); i++)
      hull_poses.push_back(poses[hull_positions[i]]);
  }

  contours_.push_back(hull_points);
  poses_contours_.push_back(hull_poses);
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop