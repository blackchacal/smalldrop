// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file detect_wound.cpp
 * \brief ROS node to detect wounds using depth camera data.
 */

#include <ros/ros.h>

#include <smalldrop_vision/camera_d415.h>
#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>

#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Dense>

using namespace smalldrop::smalldrop_vision;
using namespace smalldrop::smalldrop_segmentation;

/**
 * Global variables
 *********************************************************************************************/

camera_topics_t camera_topics;

/**
 * Function prototypes
 *********************************************************************************************/

void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform);

/**
 * Main
 *********************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_wound");

  camera_topics.rgb_info_topic = "/smalldrop/vision/camera/color/camera_info";
  camera_topics.rgb_image_topic = "/smalldrop/vision/camera/color/image_raw";
  camera_topics.ir1_info_topic = "/smalldrop/vision/camera/ir/camera_info";
  camera_topics.ir1_image_topic = "/smalldrop/vision/camera/ir/image_raw";
  camera_topics.ir2_info_topic = "/smalldrop/vision/camera/ir2/camera_info";
  camera_topics.ir2_image_topic = "/smalldrop/vision/camera/ir2/image_raw";
  camera_topics.depth_info_topic = "/smalldrop/vision/camera/depth/camera_info";
  camera_topics.depth_image_topic = "/smalldrop/vision/camera/depth/image_raw";
  camera_topics.rgb_pcloud_topic = "/smalldrop/vision/camera/point_cloud/points";

  double distance = 0.702;
  double wsp_w = 0.838 * distance; 
  double wsp_h = 0.75 * wsp_w;
  img_wsp_calibration_t calibration_data = {
    .img_width = 640,
    .img_height = 480,
    .wsp_x_min = -wsp_w/2,
    .wsp_x_max = wsp_w/2,
    .wsp_y_min = -wsp_h/2 + 0.075,
    .wsp_y_max = wsp_h/2 + 0.075
  };

  tf::TransformListener tf_listener;
  Eigen::Matrix4d transform;
  transform.setIdentity();

  CameraD415 cam(true, camera_topics);
  cam.turnOn();

  ros::Rate r(100); // 100 Hz
  while (ros::ok())
  {
    getTransformToBase(tf_listener, transform);
    WSegmentCamBinary wseg(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);

    ros::spinOnce();
    r.sleep();
  }

  cam.turnOff();
  
  return 0;
}

void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform)
{
  tf::StampedTransform transformStamped;
  try {
    tf_listener.lookupTransform("panda_link0", "camera_depth_optical_frame", ros::Time(0), transformStamped);
    
    Eigen::Vector4d t_vector(transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1);
    transform.col(3) << transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1;
    transform.col(0) << transformStamped.getBasis().getColumn(0)[0], transformStamped.getBasis().getColumn(0)[1], transformStamped.getBasis().getColumn(0)[2], 0;
    transform.col(1) << transformStamped.getBasis().getColumn(1)[0], transformStamped.getBasis().getColumn(1)[1], transformStamped.getBasis().getColumn(1)[2], 0;
    transform.col(2) << transformStamped.getBasis().getColumn(2)[0], transformStamped.getBasis().getColumn(2)[1], transformStamped.getBasis().getColumn(2)[2], 0;

    // std::cout << transform << std::endl;
  } catch (tf2::TransformException &ex) {
    return;
  }
}