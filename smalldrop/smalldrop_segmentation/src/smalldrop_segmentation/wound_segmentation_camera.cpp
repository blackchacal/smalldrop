// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera.cpp
 * \brief Defines base class for camera wound segmentation algorithms.
 */

#include <smalldrop_segmentation/wound_segmentation_camera.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCam::WSegmentCam()
 */
WSegmentCam::WSegmentCam()
{
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContour() const
 */
poses_t WSegmentCam::getWoundSegmentationPosesContour(const unsigned int contour_idx) const
{
  poses_t empty_contour_;
  if (contour_idx < poses_contours_.size())
    return poses_contours_[contour_idx];
  else
    return empty_contour_;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContour() const
 */
points_t WSegmentCam::getWoundSegmentationPointsContour(const unsigned int contour_idx) const
{
  points_t empty_contour_;
  if (contour_idx < contours_.size())
    return contours_[contour_idx];
  else
    return empty_contour_;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContours() const
 */
poses_contours_t WSegmentCam::getWoundSegmentationPosesContours() const
{
  return poses_contours_;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContours() const
 */
contours_t WSegmentCam::getWoundSegmentationPointsContours() const
{
  return contours_;
}

/**
 * \copydoc WSegmentCam::contourArea(const unsigned int contour_idx) const
 */
double WSegmentCam::contourArea(const unsigned int contour_idx) const
{
  if (contour_idx < contours_.size() && contours_[contour_idx].size() >= 3)
    return cv::contourArea(contours_[contour_idx]) * convPxSq2MeterSq();
  else
    return 0;
}

/**
 * \copydoc WSegmentCam::contourPerimeter(const unsigned int contour_idx) const
 */
double WSegmentCam::contourPerimeter(const unsigned int contour_idx) const
{
  std::vector<double> px_dim = convPx2Meter();
  if (contour_idx < contours_.size() && contours_[contour_idx].size() > 1)
    return cv::arcLength(contours_[contour_idx], true) * px_dim[0];
  else
    return 0;
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief WSegmentCam::convPx2Meter() const
 */
std::vector<double> WSegmentCam::convPx2Meter() const
{
  // TODO: get pixel dimension
  std::vector<double> px_dim;

  return px_dim;
}

/**
 * \copybrief WSegmentCam::convPxSq2MeterSq() const
 */
double WSegmentCam::convPxSq2MeterSq() const
{
  std::vector<double> px_dim = convPx2Meter();
  return px_dim[0]*px_dim[1];
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop