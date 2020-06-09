// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file camera_d415.cpp
 * \brief Defines class for Intel RealSense D415 depth camera.
 */

#include <smalldrop_vision/camera_d415.h>
#include <iostream>

namespace smalldrop
{
namespace smalldrop_vision
{
/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

/**
 * \copybrief CameraD415::CameraD415(const bool is_simulation, const camera_topics_t topics)
 */
CameraD415::CameraD415(const bool is_simulation, const camera_topics_t topics)
  : is_sim_(is_simulation), connected_(false)
{
  rgb_info_topic_ = topics.rgb_info_topic;
  rgb_image_topic_ = topics.rgb_image_topic;
  ir1_info_topic_ = topics.ir1_info_topic;
  ir1_image_topic_ = topics.ir1_image_topic;
  ir2_info_topic_ = topics.ir2_info_topic;
  ir2_image_topic_ = topics.ir2_image_topic;
  depth_info_topic_ = topics.depth_info_topic;
  depth_image_topic_ = topics.depth_image_topic;
  rgb_pcloud_topic_ = topics.rgb_pcloud_topic;

  subscribeToCameraTopics();
}

/**
 * \copydoc ICamera::turnOn()
 */
bool CameraD415::turnOn()
{
  connect();
  return isConnected();
}

/**
 * \copydoc ICamera::turnOff()
 */
bool CameraD415::turnOff()
{
  disconnect();
  return !isConnected();
}

/**
 * \copydoc ICamera::isConnected()
 */
bool CameraD415::isConnected() const
{
  return connected_;
}

/**
 * \copydoc ICamera::calibrate()
 */
bool CameraD415::calibrate()
{
  // TODO: Camera calibration
  return true;
}

/**
 * \copybrief CameraD415::getRGBInfo() const
 */
sensor_msgs::CameraInfo CameraD415::getRGBInfo() const
{
  return rgb_info_;
}

/**
 * \copybrief CameraD415::getRGBImage() const
 */
sensor_msgs::Image CameraD415::getRGBImage() const
{
  return rgb_curr_image_raw_;
}

/**
 * \copybrief CameraD415::getIR1Info() const
 */
sensor_msgs::CameraInfo CameraD415::getIR1Info() const
{
  return ir1_info_;
}

/**
 * \copybrief CameraD415::getIR1Image() const
 */
sensor_msgs::Image CameraD415::getIR1Image() const
{
  return ir1_curr_image_raw_;
}

/**
 * \copybrief CameraD415::getIR2Info() const
 */
sensor_msgs::CameraInfo CameraD415::getIR2Info() const
{
  return ir2_info_;
}

/**
 * \copybrief CameraD415::getIR2Image() const
 */
sensor_msgs::Image CameraD415::getIR2Image() const
{
  return ir2_curr_image_raw_;
}

/**
 * \copybrief CameraD415::getDepthInfo() const
 */
sensor_msgs::CameraInfo CameraD415::getDepthInfo() const
{
  return depth_info_;
}

/**
 * \copybrief CameraD415::getDepthImage() const
 */
sensor_msgs::Image CameraD415::getDepthImage() const
{
  return depth_curr_image_raw_;
}

/**
 * \copybrief CameraD415::getPointCloud() const
 */
PointCloud CameraD415::getPointCloud() const
{
  return point_cloud_;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief CameraD415::connect()
 */
void CameraD415::connect()
{
  connected_ = true;
}

/**
 * \copybrief CameraD415::disconnect()
 */
void CameraD415::disconnect()
{
  connected_ = false;
}

/**
 * \copybrief CameraD415::subscribeToCameraTopics()
 * \brief Subscribes to the camera topics for data reading.
 */
void CameraD415::subscribeToCameraTopics()
{
  rgb_info_topic_sub_ =
      nh_.subscribe<sensor_msgs::CameraInfo>(rgb_info_topic_, 10, &CameraD415::getRGBInfoFromTopic, this);
  rgb_image_topic_sub_ =
      nh_.subscribe<sensor_msgs::Image>(rgb_image_topic_, 10, &CameraD415::getRGBImageFromTopic, this);
  ir1_info_topic_sub_ =
      nh_.subscribe<sensor_msgs::CameraInfo>(ir1_info_topic_, 10, &CameraD415::getIR1InfoFromTopic, this);
  ir1_image_topic_sub_ =
      nh_.subscribe<sensor_msgs::Image>(ir1_image_topic_, 10, &CameraD415::getIR1ImageFromTopic, this);
  ir2_info_topic_sub_ =
      nh_.subscribe<sensor_msgs::CameraInfo>(ir2_info_topic_, 10, &CameraD415::getIR2InfoFromTopic, this);
  ir2_image_topic_sub_ =
      nh_.subscribe<sensor_msgs::Image>(ir2_image_topic_, 10, &CameraD415::getIR2ImageFromTopic, this);
  depth_info_topic_sub_ =
      nh_.subscribe<sensor_msgs::CameraInfo>(depth_info_topic_, 10, &CameraD415::getDepthInfoFromTopic, this);
  depth_image_topic_sub_ =
      nh_.subscribe<sensor_msgs::Image>(depth_image_topic_, 10, &CameraD415::getDepthImageFromTopic, this);
  rgb_pcloud_topic_sub_ =
      nh_.subscribe<PointCloud>(rgb_pcloud_topic_, 10, &CameraD415::getPointCloudFromTopic, this);
}

/**
 * \copybrief CameraD415::getRGBInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
 * \brief Get RGB camera info from topic.
 */
void CameraD415::getRGBInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  if (isConnected())
    rgb_info_ = *msg;
}

/**
 * \copybrief CameraD415::getRGBImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
 * \brief Get RGB camera image from topic.
 */
void CameraD415::getRGBImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
{
  if (isConnected())
    rgb_curr_image_raw_ = *msg;
}

/**
 * \copybrief CameraD415::getIR1InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
 * \brief Get IR1 camera info from topic.
 */
void CameraD415::getIR1InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  if (isConnected())
    ir1_info_ = *msg;
}

/**
 * \copybrief CameraD415::getIR1ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
 * \brief Get IR1 camera image from topic.
 */
void CameraD415::getIR1ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
{
  if (isConnected())
    ir1_curr_image_raw_ = *msg;
}

/**
 * \copybrief CameraD415::getIR2InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
 * \brief Get IR2 camera info from topic.
 */
void CameraD415::getIR2InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  if (isConnected())
    ir2_info_ = *msg;
}

/**
 * \copybrief CameraD415::getIR2ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
 * \brief Get IR2 camera image from topic.
 */
void CameraD415::getIR2ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
{
  if (isConnected())
    ir2_curr_image_raw_ = *msg;
}

/**
 * \copybrief CameraD415::getDepthInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
 * \brief Get Depth camera info from topic.
 */
void CameraD415::getDepthInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
{
  if (isConnected())
    depth_info_ = *msg;
}

/**
 * \copybrief CameraD415::getDepthImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
 * \brief Get Depth camera image from topic.
 */
void CameraD415::getDepthImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
{
  if (isConnected())
    depth_curr_image_raw_ = *msg;
}

/**
 * \copybrief CameraD415::getPointCloudFromTopic(const PointCloud::ConstPtr &msg)
 */
void CameraD415::getPointCloudFromTopic(const PointCloud::ConstPtr &msg)
{
  if (isConnected())
    point_cloud_ = *msg;
}

}  // namespace smalldrop_vision

}  // namespace smalldrop