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
#include <pcl_ros/point_cloud.h>

#include <tf2_ros/transform_listener.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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
   * \fn WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image& depth_image, const PointCloud& point_cloud) 
   * \brief Default constructor
   *
   * \param rgb_image RGB image from were the wound will be segmented.
   * \param depth_image Depth image from were the wound will be segmented.
   * \param point_cloud Point Cloud from were the wound will be segmented.
   * \param transform Transform matrix from camera depth frame and robot base frame 
   * \param calibration_data Data for image-workspace calibration.
   */
  WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image& depth_image,
                    const PointCloud& point_cloud, const Eigen::Matrix4d& transform, const img_wsp_calibration_t calibration_data);

  ~WSegmentCamBinary()
  {
  }
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_CAMERA_BINARIZATION_H