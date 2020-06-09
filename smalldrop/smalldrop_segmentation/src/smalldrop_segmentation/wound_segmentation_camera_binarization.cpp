// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera_binarization.cpp
 * \brief Defines class for camera wound segmentation binarization algorithm.
 */

#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCamBinary::WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image&
 * depth_image, const PointCloud& point_cloud, const Eigen::Matrix4d& transform, const img_wsp_calibration_t calibration_data)
 */
WSegmentCamBinary::WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image& depth_image,
                                     const PointCloud& point_cloud, const Eigen::Matrix4d& transform, const img_wsp_calibration_t calibration_data)
  : WSegmentCam(calibration_data)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    // ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat rsize(340, 600, CV_8UC3);
  rsize = cv::Scalar::all(0);
  cv::resize(cv_ptr->image, rsize, rsize.size());

  // Set to grayscale
  cv::Mat grey;
  cv::cvtColor(rsize, grey, CV_BGR2GRAY);

  // Binarize image
  /*
    Threshold Type
    0: Binary
    1: Binary Inverted
    2: Threshold Truncated
    3: Threshold to Zero
    4: Threshold to Zero Inverted
   */
  cv::Mat bin;
  int threshold_value = 10;
  int threshold_type = 1;
  int const max_BINARY_value = 255;
  cv::threshold(grey, bin, threshold_value, max_BINARY_value, threshold_type);

  // Get contours
  // contours_t contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(bin, contours_, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
  cv::drawContours(rsize, contours_, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

  if (contours_.size() == 0)
    return;

  // Get points inside contours
  getContoursRegion();

  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits (wsp_x_min_limit_, wsp_x_max_limit_);
  pass.filter(*cloud_filtered_ptr);
  
  pass.setInputCloud(cloud_filtered_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits (wsp_y_min_limit_, wsp_y_max_limit_);
  pass.filter(*cloud_filtered_ptr);

  pass.setInputCloud(cloud_filtered_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits (0.3, 0.75);
  pass.filter(*cloud_filtered_ptr);
  
  // Get poses inside contours
  getPosesContoursRegion(*cloud_filtered_ptr, transform);
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop