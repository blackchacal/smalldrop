// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera_binarization.h
 * \brief Declares class for camera wound segmentation binarization algorithm.
 */

#ifndef _SMALLDROP_WOUND_SEGMENTATION_CAMERA_BINARIZATION_H
#define _SMALLDROP_WOUND_SEGMENTATION_CAMERA_BINARIZATION_H

#include <smalldrop_segmentation/wound_segmentation_camera.h>

// ROS messages
#include <sensor_msgs/Image.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/**
 * \class WSegmentCamBinary
 * \brief Class for camera wound segmentation binarization algorithm.
 */
class WSegmentCamBinary : public WSegmentCam
{
public:
  /**
   * Class Constructors & Destructors
   *****************************************************************************************/

  /**
   * \fn WSegmentCamBinary(const sensor_msgs::Image& rgb_image) 
   * \brief Default constructor
   *
   * \param rgb_image RGB image from were the wound will be segmented.
   */
  WSegmentCamBinary(const sensor_msgs::Image& rgb_image);

  ~WSegmentCamBinary()
  {
  }
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_CAMERA_BINARIZATION_H