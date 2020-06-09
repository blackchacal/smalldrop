// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file i_wound_segmentation.h
 * \brief Declares interface for wound segmentation algorithms.
 */

#ifndef _SMALLDROP_INTERFACE_WOUND_SEGMENTATION_H
#define _SMALLDROP_INTERFACE_WOUND_SEGMENTATION_H

// ROS messages
#include <geometry_msgs/Pose.h>

// Libraries
#include "opencv2/imgproc.hpp"

namespace smalldrop
{
namespace smalldrop_segmentation
{
/**
 * \typedef pose_t
 */
typedef geometry_msgs::Pose pose_t;

/**
 * \typedef poses_t
 */
typedef std::vector<pose_t> poses_t;

/**
 * \typedef poses_contours_t
 */
typedef std::vector<poses_t> poses_contours_t;


/**
 * \typedef point_t
 */
typedef cv::Point point_t;

/**
 * \typedef points_t
 */
typedef std::vector<point_t> points_t;

/**
 * \typedef contours_t
 */
typedef std::vector<points_t> contours_t;

/**
 * \typedef img_wsp_calibration_t
 */
typedef struct
{
  unsigned int img_width;  /** Image width in px. */
  unsigned int img_height; /** Image height in px. */
  double wsp_x_min;        /** Robot workspace coordinates minimum x limit. */
  double wsp_x_max;        /** Robot workspace coordinates maximum x limit. */
  double wsp_y_min;        /** Robot workspace coordinates minimum y limit. */
  double wsp_y_max;        /** Robot workspace coordinates maximum y limit. */
} img_wsp_calibration_t;

/**
 * \class IWoundSegmentation
 * \brief Interface for wound segmentation algorithms.
 */
class IWoundSegmentation
{
public:
  virtual ~IWoundSegmentation()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn virtual poses_t getWoundSegmentationPosesContour(const unsigned int contour_idx) const
   * \brief Returns a vector with wound segmentation contour poses.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  virtual poses_t getWoundSegmentationPosesContour(const unsigned int contour_idx) const = 0;

  /**
   * \fn virtual points_t getWoundSegmentationPointsContour(const unsigned int contour_idx) const
   * \brief Returns a vector with wound segmentation contour points.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  virtual points_t getWoundSegmentationPointsContour(const unsigned int contour_idx) const = 0;

  /**
   * \fn virtual contours_t getWoundSegmentationPosesContours() const
   * \brief Returns a vector with detected wound segmentation contours, using poses.
   */
  virtual poses_contours_t getWoundSegmentationPosesContours() const = 0;

  /**
   * \fn virtual contours_t getWoundSegmentationPointsContours() const
   * \brief Returns a vector with detected wound segmentation contours, using points.
   */
  virtual contours_t getWoundSegmentationPointsContours() const = 0;

  /**
   * \fn virtual double contourArea(const unsigned int contour_idx) const
   * \brief Calculates the chosen contour area.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  virtual double contourArea(const unsigned int contour_idx) const = 0;

  /**
   * \fn virtual double contourPerimeter(const unsigned int contour_idx) const
   * \brief Calculates the chosen contour perimeter.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  virtual double contourPerimeter(const unsigned int contour_idx) const = 0;
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_INTERFACE_WOUND_SEGMENTATION_H