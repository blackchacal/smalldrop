// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file detect_wound.cpp
 * \brief ROS node to detect wounds using depth camera data.
 */

#include <ros/ros.h>

#include <smalldrop_vision/camera_d415.h>
#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>

using namespace smalldrop::smalldrop_vision;
using namespace smalldrop::smalldrop_segmentation;

/**
 * Global variables
 *********************************************************************************************/

camera_topics_t camera_topics;

/**
 * Function prototypes
 *********************************************************************************************/



/**
 * Main
 *********************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_wound");

  camera_topics.rgb_info_topic = "/realsense_plugin/camera/color/camera_info";
  camera_topics.rgb_image_topic = "/realsense_plugin/camera/color/image_raw";
  camera_topics.ir1_info_topic = "/realsense_plugin/camera/ir/camera_info";
  camera_topics.ir1_image_topic = "/realsense_plugin/camera/ir/image_raw";
  camera_topics.ir2_info_topic = "/realsense_plugin/camera/ir2/camera_info";
  camera_topics.ir2_image_topic = "/realsense_plugin/camera/ir2/image_raw";
  camera_topics.depth_info_topic = "/realsense_plugin/camera/depth/camera_info";
  camera_topics.depth_image_topic = "/realsense_plugin/camera/depth/image_raw";
  camera_topics.rgb_pcloud_topic = "/realsense_plugin/camera/point_cloud/data";

  CameraD415 cam(true, camera_topics);
  cam.turnOn();

  ros::Rate r(100); // 100 Hz
  while (ros::ok())
  {
    sensor_msgs::Image rgb_img = cam.getRGBImage();
    WSegmentCamBinary wseg(rgb_img);

    ros::spinOnce();
    r.sleep();
  }

  cam.turnOff();
  
  return 0;
}
