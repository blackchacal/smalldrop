// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file camera_d415.h
 * \brief Declares class for Intel RealSense D415 depth camera.
 */

#ifndef _SMALLDROP_CAMERA_D415_H
#define _SMALLDROP_CAMERA_D415_H

#include <ros/ros.h>

#include <smalldrop_vision/i_camera.h>

// ROS messages
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

namespace smalldrop
{
namespace smalldrop_vision
{

typedef struct {
  std::string rgb_info_topic;
  std::string rgb_image_topic;
  std::string ir1_info_topic;
  std::string ir1_image_topic;
  std::string ir2_info_topic;
  std::string ir2_image_topic;
  std::string depth_info_topic;
  std::string depth_image_topic;
  std::string rgb_pcloud_topic;
} camera_topics_t;

/**
 * \class CameraD415
 * \brief Interface for general cameras.
 */
class CameraD415 : public ICamera
{
public:
  CameraD415(const bool is_simulation, const camera_topics_t topics);

  ~CameraD415()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \copydoc ICamera::turnOn()
   */
  virtual bool turnOn() override;

  /**
   * \copydoc ICamera::turnOff()
   */
  virtual bool turnOff() override;

  /**
   * \copydoc ICamera::isConnected()
   */
  virtual bool isConnected() const override;

  /**
   * \copydoc ICamera::calibrate()
   */
  virtual bool calibrate() override;

  /**
   * \fn sensor_msgs::CameraInfo getRGBInfo() const
   * \brief Return the RGB camera info.
   */
  sensor_msgs::CameraInfo getRGBInfo() const;

  /**
   * \fn sensor_msgs::Image getRGBImage() const
   * \brief Return the RGB camera image.
   */
  sensor_msgs::Image getRGBImage() const;

  /**
   * \fn sensor_msgs::CameraInfo getIR1Info() const
   * \brief Return the IR1 camera info.
   */
  sensor_msgs::CameraInfo getIR1Info() const;

  /**
   * \fn sensor_msgs::Image getIR1Image() const
   * \brief Return the IR1 camera image.
   */
  sensor_msgs::Image getIR1Image() const;

  /**
   * \fn sensor_msgs::CameraInfo getIR2Info() const
   * \brief Return the IR2 camera info.
   */
  sensor_msgs::CameraInfo getIR2Info() const;

  /**
   * \fn sensor_msgs::Image getIR2Image() const
   * \brief Return the IR2 camera image.
   */
  sensor_msgs::Image getIR2Image() const;

  /**
   * \fn sensor_msgs::CameraInfo getDepthInfo() const
   * \brief Return the Depth camera info.
   */
  sensor_msgs::CameraInfo getDepthInfo() const;

  /**
   * \fn sensor_msgs::Image getDepthImage() const
   * \brief Return the Depth camera image.
   */
  sensor_msgs::Image getDepthImage() const;

  /**
   * \fn sensor_msgs::PointCloud2 getPointCloud() const
   * \brief Return point cloud data.
   */
  sensor_msgs::PointCloud2 getPointCloud() const;

private:
  /**
   * Class members
   *****************************************************************************************/

  bool is_sim_;    /** \var Indicates if camera is operating in simulation mode or real mode. */
  bool connected_; /** \var Indicates the connection state of the camera. */

  // Camera data
  sensor_msgs::CameraInfo rgb_info_;      /** \var RGB camera information. */
  sensor_msgs::Image rgb_curr_image_raw_; /** \var Current RGB camera image. */

  sensor_msgs::CameraInfo ir1_info_;      /** \var IR1 camera information. */
  sensor_msgs::Image ir1_curr_image_raw_; /** \var Current RGB camera image. */

  sensor_msgs::CameraInfo ir2_info_;      /** \var IR2 camera information. */
  sensor_msgs::Image ir2_curr_image_raw_; /** \var Current RGB camera image. */

  sensor_msgs::CameraInfo depth_info_;      /** \var Depth camera information. */
  sensor_msgs::Image depth_curr_image_raw_; /** \var Current RGB camera image. */

  sensor_msgs::PointCloud2 point_cloud_; /** \var Point cloud obtained from depth data. */

  // ROS Topics
  std::string rgb_info_topic_;
  std::string rgb_image_topic_;
  std::string ir1_info_topic_;
  std::string ir1_image_topic_;
  std::string ir2_info_topic_;
  std::string ir2_image_topic_;
  std::string depth_info_topic_;
  std::string depth_image_topic_;
  std::string rgb_pcloud_topic_;

  ros::Subscriber rgb_info_topic_sub_;
  ros::Subscriber rgb_image_topic_sub_;
  ros::Subscriber ir1_info_topic_sub_;
  ros::Subscriber ir1_image_topic_sub_;
  ros::Subscriber ir2_info_topic_sub_;
  ros::Subscriber ir2_image_topic_sub_;
  ros::Subscriber depth_info_topic_sub_;
  ros::Subscriber depth_image_topic_sub_;
  ros::Subscriber rgb_pcloud_topic_sub_;

  ros::NodeHandle nh_; /** \var ROS node handle to access topics system. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn void connect()
   * \brief Establishes the connection with the camera.
   */
  void connect();

  /**
   * \fn void disconnect()
   * \brief Disconnects the camera.
   */
  void disconnect();

  /**
   * \fn void subscribeToCameraTopics()
   * \brief Subscribes to the camera topics for data reading.
   */
  void subscribeToCameraTopics();

  /**
   * \fn void getRGBInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
   * \brief Get RGB camera info from topic.
   */
  void getRGBInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg);

  /**
   * \fn void getRGBImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
   * \brief Get RGB camera image from topic.
   */
  void getRGBImageFromTopic(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * \fn void getIR1InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
   * \brief Get IR1 camera info from topic.
   */
  void getIR1InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg);

  /**
   * \fn void getIR1ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
   * \brief Get IR1 camera image from topic.
   */
  void getIR1ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * \fn void getIR2InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
   * \brief Get IR2 camera info from topic.
   */
  void getIR2InfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg);

  /**
   * \fn void getIR2ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
   * \brief Get IR2 camera image from topic.
   */
  void getIR2ImageFromTopic(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * \fn void getDepthInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg)
   * \brief Get Depth camera info from topic.
   */
  void getDepthInfoFromTopic(const sensor_msgs::CameraInfo::ConstPtr &msg);

  /**
   * \fn void getDepthImageFromTopic(const sensor_msgs::Image::ConstPtr &msg)
   * \brief Get Depth camera image from topic.
   */
  void getDepthImageFromTopic(const sensor_msgs::Image::ConstPtr &msg);

  /**
   * \fn void getPointCloudFromTopic(const sensor_msgs::PointCloud2::ConstPtr &msg)
   * \brief Get point cloud data from topic.
   */
  void getPointCloudFromTopic(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

}  // namespace smalldrop_vision
}  // namespace smalldrop

#endif  // _SMALLDROP_CAMERA_D415_H