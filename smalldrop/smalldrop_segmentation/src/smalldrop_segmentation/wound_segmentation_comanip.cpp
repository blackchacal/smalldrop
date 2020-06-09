// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_comanip.cpp
 * \brief Defines base class for co-manipulation wound segmentation algorithms.
 */

#include <smalldrop_segmentation/wound_segmentation_comanip.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCoManip::WSegmentCoManip(const std::string filepath)
 */
WSegmentCoManip::WSegmentCoManip(const std::string filepath)
  : image_width_(500)
  , image_height_(500)
  , wsp_x_min_limit_(0.0)
  , wsp_x_max_limit_(1.0)
  , wsp_y_min_limit_(-0.5)
  , wsp_y_max_limit_(0.5)
  , filepath_(filepath)
{
}

/**
 * \copybrief WSegmentCoManip::WSegmentCoManip(const std::string filepath, const img_wsp_calibration_t calibration_data)
 */
WSegmentCoManip::WSegmentCoManip(const std::string filepath, const img_wsp_calibration_t calibration_data) : filepath_(filepath)
{
  filepath_ = filepath;
  image_width_ = calibration_data.img_width;
  image_height_ = calibration_data.img_height;
  wsp_x_min_limit_ = calibration_data.wsp_x_min;
  wsp_x_max_limit_ = calibration_data.wsp_x_max;
  wsp_y_min_limit_ = calibration_data.wsp_y_min;
  wsp_y_max_limit_ = calibration_data.wsp_y_max;
}

/**
 * \copybrief WSegmentCoManip::getWoundSegmentationPosesContour() const
 */
poses_t WSegmentCoManip::getWoundSegmentationPosesContour(const unsigned int contour_idx) const
{
  poses_t empty_contour_;
  if (contour_idx < poses_contours_.size())
    return poses_contours_[contour_idx];
  else
    return empty_contour_;
}

/**
 * \copybrief WSegmentCoManip::getWoundSegmentationPointsContour() const
 */
points_t WSegmentCoManip::getWoundSegmentationPointsContour(const unsigned int contour_idx) const
{
  points_t empty_contour_;
  if (contour_idx < contours_.size())
    return contours_[contour_idx];
  else
    return empty_contour_;
}

/**
 * \copybrief WSegmentCoManip::getWoundSegmentationPosesContours() const
 */
poses_contours_t WSegmentCoManip::getWoundSegmentationPosesContours() const
{
  return poses_contours_;
}

/**
 * \copybrief WSegmentCoManip::getWoundSegmentationPointsContours() const
 */
contours_t WSegmentCoManip::getWoundSegmentationPointsContours() const
{
  return contours_;
}

/**
 * \copydoc WSegmentCoManip::contourArea(const unsigned int contour_idx) const
 */
double WSegmentCoManip::contourArea(const unsigned int contour_idx) const
{
  if (contour_idx < contours_.size() && contours_[contour_idx].size() >= 3)
    return cv::contourArea(contours_[contour_idx]) * convPxSq2MeterSq();
  else
    return 0;
}

/**
 * \copydoc WSegmentCoManip::contourPerimeter(const unsigned int contour_idx) const
 */
double WSegmentCoManip::contourPerimeter(const unsigned int contour_idx) const
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

/**
 * \copybrief WSegmentCoManip::convPoseToPoint() const
 */
point_t WSegmentCoManip::convPoseToPoint(pose_t pose) const
{
  point_t pt;
  pt.x = round((image_width_ / (wsp_y_max_limit_ - wsp_y_min_limit_)) * (pose.position.y - wsp_y_min_limit_));
  pt.y = round((image_height_ / (wsp_x_max_limit_ - wsp_x_min_limit_)) * (pose.position.x - wsp_x_min_limit_));
  return pt;
}

/**
 * \copybrief WSegmentCoManip::loadWoundSegmentationPoses()
 */
poses_t WSegmentCoManip::loadWoundSegmentationPoses()
{
  std::string header;
  std::string x, y, z, ox, oy, oz, ow;
  poses_t poses;

  fh_.open(filepath_, std::fstream::in);
  if (fh_.is_open())
  {
    getline(fh_, header);
    while (getline(fh_, x, ' '))
    {
      getline(fh_, y, ' ');
      getline(fh_, z, ' ');
      getline(fh_, ox, ' ');
      getline(fh_, oy, ' ');
      getline(fh_, oz, ' ');
      getline(fh_, ow);
      pose_t pose;
      pose.position.x = std::stod(x);
      pose.position.y = std::stod(y);
      pose.position.z = std::stod(z);
      pose.orientation.x = std::stod(ox);
      pose.orientation.y = std::stod(oy);
      pose.orientation.z = std::stod(oz);
      pose.orientation.w = std::stod(ow);
      poses.push_back(pose);
    }
    fh_.close();
  }
  else
    ROS_ERROR("The file was not properly opened!");

  return poses;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief WSegmentCoManip::convPx2Meter() const
 */
std::vector<double> WSegmentCoManip::convPx2Meter() const
{
  double px_w = (wsp_y_max_limit_ - wsp_y_min_limit_) / image_width_;
  double px_h = (wsp_x_max_limit_ - wsp_x_min_limit_) / image_height_;
  std::vector<double> px_dim;
  px_dim.push_back(px_w);
  px_dim.push_back(px_h);

  return px_dim;
}

/**
 * \copybrief WSegmentCoManip::convPxSq2MeterSq() const
 */
double WSegmentCoManip::convPxSq2MeterSq() const
{
  std::vector<double> px_dim = convPx2Meter();
  return px_dim[0]*px_dim[1];
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop