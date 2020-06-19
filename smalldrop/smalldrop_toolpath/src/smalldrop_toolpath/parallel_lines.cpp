// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file parallel_lines.cpp
 * \brief Defines ParallelLines class for robot arm wound filling printing path planning.
 */

#include <smalldrop_toolpath/parallel_lines.h>

namespace smalldrop
{
namespace smalldrop_toolpath
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief ParallelLines::ParallelLines(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis,
 const double pose_z, const img_wsp_calibration_t calibration_data)
 */
ParallelLines::ParallelLines(const points_t contour, const unsigned int offset, const IMAGE_AXIS axis,
                             const double pose_z, const img_wsp_calibration_t calibration_data)
  : ToolPath(calibration_data)
{
  cv::Rect bounding_box = cv::boundingRect(contour);
  std::vector<points_t> lines = getGridLines(bounding_box, offset, axis);

  cv::Mat img(image_width_, image_height_, CV_8UC1);
  img = cv::Scalar::all(0);

  // Get complex contours to properly intersection calculation with grid lines
  contours_t complex_contours;
  getComplexContours(contour, complex_contours, img);

  if (complex_contours.size() == 0)
    return;

  // For every grid line find the interception points with the contour.
  // Order the points so that it follows a parallel path
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::LineIterator it(img, lines[i].front(), lines[i].back(), 4);
    points_t line_interceptions;
    for (int j = 0; j < it.count; j++, ++it)
    {
      if (cv::pointPolygonTest(complex_contours[0], it.pos(), false) == 0)
        line_interceptions.push_back(it.pos());
    }
    // Only select the two extreme interception points
    if (line_interceptions.size() == 1)
      points_.push_back(line_interceptions.front());
    else if (line_interceptions.size() > 1)
    {
      // If i is odd switch the order of the points
      if (i % 2 == 0)
      {
        points_.push_back(line_interceptions.front());
        points_.push_back(line_interceptions.back());
      }
      else
      {
        points_.push_back(line_interceptions.back());
        points_.push_back(line_interceptions.front());
      }
    }
  }

  poses_ = convPathPointToPose(points_, pose_z);

  // Add printing actions to path points
  for (unsigned int i = 0; i < poses_.size(); i++)
  {
    if (i == 0)
      actions_.push_back(PRINT_ACTION::START);  // First pose
    else if (i == poses_.size() - 1)
      actions_.push_back(PRINT_ACTION::STOP);  // Last pose
    else
      actions_.push_back(PRINT_ACTION::CONTINUE);  // Last pose
  }

  // Calculate path length.
  calcLength();
}

/**
 * \copybrief ParallelLines::ParallelLines(const points_t contour, const poses_t poses_contour_region, const
 * Eigen::Matrix4d& transform, const unsigned int offset, const IMAGE_AXIS axis, const img_wsp_calibration_t
 * calibration_data)
 */
ParallelLines::ParallelLines(const points_t contour, const poses_t poses_contour_region,
                             const Eigen::Matrix4d& transform, const unsigned int offset, const IMAGE_AXIS axis,
                             const img_wsp_calibration_t calibration_data)
  : ToolPath(calibration_data)
{
  cv::Rect bounding_box = cv::boundingRect(contour);
  std::vector<points_t> lines = getGridLines(bounding_box, offset, axis);

  cv::Mat img(image_width_, image_height_, CV_8UC1);
  img = cv::Scalar::all(0);

  // Get complex contours to properly intersection calculation with grid lines
  contours_t complex_contours;
  getComplexContours(contour, complex_contours, img);

  if (complex_contours.size() == 0)
    return;

  // For every grid line find the interception points with the contour.
  // Order the points so that it follows a parallel path
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::LineIterator it(img, lines[i].front(), lines[i].back(), 4);
    points_t line_interceptions;
    for (int j = 0; j < it.count; j++, ++it)
    {
      if (cv::pointPolygonTest(complex_contours[0], it.pos(), false) == 0)
        line_interceptions.push_back(it.pos());
    }
    // Only select the two extreme interception points
    if (line_interceptions.size() == 1)
      points_.push_back(line_interceptions.front());
    else if (line_interceptions.size() > 1)
    {
      // If i is odd switch the order of the points
      if (i % 2 == 0)
      {
        points_.push_back(line_interceptions.front());
        points_.push_back(line_interceptions.back());
      }
      else
      {
        points_.push_back(line_interceptions.back());
        points_.push_back(line_interceptions.front());
      }
    }
  }

  poses_ = convPathPointToPose(points_, poses_contour_region, transform);

  // Add printing actions to path points
  for (unsigned int i = 0; i < poses_.size(); i++)
  {
    if (i == 0)
      actions_.push_back(PRINT_ACTION::START);  // First pose
    else if (i == poses_.size() - 1)
      actions_.push_back(PRINT_ACTION::STOP);  // Last pose
    else
      actions_.push_back(PRINT_ACTION::CONTINUE);  // Last pose
  }

  // Calculate path length.
  calcLength();
}

}  // namespace smalldrop_toolpath

}  // namespace smalldrop