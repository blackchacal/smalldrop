// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file toolpath.cpp
 * \brief Defines ToolPath class for robot arm printing path planning.
 */

#include <smalldrop_toolpath/toolpath.h>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrieef ToolPath(const img_wsp_calibration_t calibration_data)
 */
ToolPath::ToolPath(const img_wsp_calibration_t calibration_data) : Path()
{
  image_width_ = calibration_data.img_width;
  image_height_ = calibration_data.img_height;
  wsp_x_min_limit_ = calibration_data.wsp_x_min;
  wsp_x_max_limit_ = calibration_data.wsp_x_max;
  wsp_y_min_limit_ = calibration_data.wsp_y_min;
  wsp_y_max_limit_ = calibration_data.wsp_y_max;
}

/**
 * \copybrief ToolPath::actions() const
 */
path_actions_t ToolPath::actions() const
{
  return actions_;
}

/**
 * \copybrief ToolPath::points() const
 */
points_t ToolPath::points() const
{
  return points_;
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \copybrief ToolPath::getGridLines() const
 */
std::vector<points_t> ToolPath::getGridLines(cv::Rect bounding_box, unsigned int offset, IMAGE_AXIS axis) const
{
  unsigned int increment = 0;
  unsigned int nlines;
  unsigned int padding = 10;
  std::vector<points_t> lines;

  if (offset == 0)
    offset = 5;

  if (axis == IMAGE_AXIS::X)
    nlines = ceil((bounding_box.height + 2 * padding) / offset);
  else
    nlines = ceil((bounding_box.width + 2 * padding) / offset);

  for (size_t i = 0; i <= nlines; i++)
  {
    points_t line;
    if (axis == IMAGE_AXIS::X)
    {
      point_t pt1(bounding_box.x - padding, bounding_box.y - padding + increment);
      line.push_back(pt1);
      point_t pt2(bounding_box.x + bounding_box.width + padding, bounding_box.y - padding + increment);
      line.push_back(pt2);
    }
    else
    {
      point_t pt1(bounding_box.x - padding + increment, bounding_box.y - padding);
      line.push_back(pt1);
      point_t pt2(bounding_box.x - padding + increment, bounding_box.y + bounding_box.height + padding);
      line.push_back(pt2);
    }
    lines.push_back(line);
    increment += offset;
  }
  return lines;
}

/**
 * \copybrief ToolPath::getComplexContours() const
 */
void ToolPath::getComplexContours(const points_t contour, contours_t &complex_contours, cv::Mat &img) const
{
  // Get new contour with more points to find more interceptions with the grid lines
  contours_t simple_contours;
  if (contour.size() >= 3) // Needs at least three points to form a close contour
  {
    simple_contours.push_back(contour);
    cv::drawContours(img, simple_contours, -1, cv::Scalar(100), 1, cv::LINE_AA);
    cv::findContours(img, complex_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  }
}

/**
 * \copybrief ToolPath::convPathPointToPose(const points_t path, const double pose_z) const
 */
poses_t ToolPath::convPathPointToPose(const points_t path, const double pose_z) const
{
  poses_t new_path;
  std::vector<double> pxDim = convPx2Meter();  // pixel dimensions in m

  for (unsigned int i = 0; i < path.size(); i++)
  {
    pose_t pose;
    pose.position.x = wsp_x_min_limit_ + path[i].y * pxDim[1];
    pose.position.y = wsp_y_min_limit_ + path[i].x * pxDim[0];
    pose.position.z = pose_z;
    pose.orientation.x = 1.0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    new_path.push_back(pose);
  }
  return new_path;
}

/**
 * \copybrief ToolPath::convPx2Meter() const
 */
std::vector<double> ToolPath::convPx2Meter() const
{
  double px_w = (wsp_y_max_limit_ - wsp_y_min_limit_) / image_width_;
  double px_h = (wsp_x_max_limit_ - wsp_x_min_limit_) / image_height_;
  std::vector<double> px_dim;
  px_dim.push_back(px_w);
  px_dim.push_back(px_h);

  return px_dim;
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop