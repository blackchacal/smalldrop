// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera.h
 * \brief Declares base class for camera wound segmentation algorithms.
 */

#ifndef _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H
#define _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H

#include <ros/ros.h>

#include <smalldrop_segmentation/i_wound_segmentation.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{

/**
 * \class WSegmentCam
 * \brief Base class for camera wound segmentation algorithms.
 */
class WSegmentCam : public IWoundSegmentation
{
public:
  virtual ~WSegmentCam()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPosesContour() const
   */
  virtual poses_t getWoundSegmentationPosesContour(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPointsContour() const
   */
  virtual points_t getWoundSegmentationPointsContour(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPosesContours() const
   */
  virtual poses_contours_t getWoundSegmentationPosesContours() const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPointsContours() const
   */
  virtual contours_t getWoundSegmentationPointsContours() const override;

  /**
   * \copydoc IWoundSegmentation::contourArea(const unsigned int contour_idx) const
   */
  virtual double contourArea(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::contourPerimeter(const unsigned int contour_idx) const
   */
  virtual double contourPerimeter(const unsigned int contour_idx) const override;

protected:
  /**
   * Class members
   *****************************************************************************************/

  poses_contours_t poses_contours_; /** \var Vector with detected wound segmentation contours using poses. */
  contours_t contours_;             /** \var Vector with detected wound segmentation contours. */

  // Robot workspace and segmentation image
  unsigned int image_width_;  /** \var Image width in px. */
  unsigned int image_height_; /** \var Image height in px. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn WSegmentCam()
   * \brief Default constructor.
   */
  WSegmentCam();

private:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn double convPx2Meter()
   * \brief Convert the pixel size to meters
   */
  std::vector<double> convPx2Meter() const;

  /**
   * \fn double convPxSq2MeterSq()
   * \brief Convert the pixel squared area to meter squared.
   */
  double convPxSq2MeterSq() const;
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H