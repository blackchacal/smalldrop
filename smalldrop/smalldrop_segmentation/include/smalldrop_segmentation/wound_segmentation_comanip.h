// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_comanip.h
 * \brief Declares base class for co-manipulation wound segmentation algorithms.
 */

#ifndef _SMALLDROP_WOUND_SEGMENTATION_COMANIP_H
#define _SMALLDROP_WOUND_SEGMENTATION_COMANIP_H

#include <ros/ros.h>

#include <smalldrop_segmentation/i_wound_segmentation.h>

// Standard Library
#include <fstream>

namespace smalldrop
{
namespace smalldrop_segmentation
{

/**
 * \class WSegmentCoManip
 * \brief Base class for co-manipulation wound segmentation algorithms.
 */
class WSegmentCoManip : public IWoundSegmentation
{
public:
  virtual ~WSegmentCoManip()
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
  poses_contours_t poses_contours_region_;  /** \var Vector with poses contained by detected wound segmentation contours */
  contours_t contours_region_;              /** \var Vector with points contained by detected wound segmentation contours */

  // Robot workspace and segmentation image
  unsigned int image_width_;  /** \var Image width in px. */
  unsigned int image_height_; /** \var Image height in px. */
  double wsp_x_min_limit_;    /** \var Robot workspace coordinates minimum x limit. */
  double wsp_x_max_limit_;    /** \var Robot workspace coordinates maximum x limit. */
  double wsp_y_min_limit_;    /** \var Robot workspace coordinates minimum y limit. */
  double wsp_y_max_limit_;    /** \var Robot workspace coordinates maximum y limit. */

  // Files
  std::string filepath_; /** \var Wound segmentation points file path. */
  std::ifstream fh_;     /** \var File handler for segmentation points loading. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn WSegmentCoManip(const std::string filepath)
   * \brief Default constructor.
   *
   * \param filepath The path to the file with wound segmentation poses data.
   */
  WSegmentCoManip(const std::string filepath);

  /**
   * \fn WSegmentCoManip(const std::string filepath, const img_wsp_calibration_t calibration_data) 
   * \brief Constructor where robot workspace and image limits are defined.
   *
   * \param filepath The path to the file with wound segmentation poses data.
   * \param calibration_data Data for image-workspace calibration.
   */
  WSegmentCoManip(const std::string filepath, const img_wsp_calibration_t calibration_data);

  /**
   * \fn point_t convPoseToPoint(pose_t pose) const
   * \brief Convert the pose coordinates to the image pixel coordinates.
   *
   * \param pose A pose_t that is going to be converted to point_t.
   */
  point_t convPoseToPoint(pose_t pose) const;

  /**
   * \fn poses_t loadWoundSegmentationPoses()
   * \brief Loads the wound segmentation points defined during co-manipulation. The points are stored on a file.
   */
  poses_t loadWoundSegmentationPoses();

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

#endif  // _SMALLDROP_WOUND_SEGMENTATION_COMANIP_H