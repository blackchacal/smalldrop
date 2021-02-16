// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file detect_wound.cpp
 * \brief ROS node to detect wounds using depth camera data.
 */

#include <ros/ros.h>

#include <smalldrop_vision/camera_d415.h>
#include <smalldrop_segmentation/wound_segmentation_camera.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace smalldrop::smalldrop_vision;
using namespace smalldrop::smalldrop_segmentation;

/**
 * Global variables
 *********************************************************************************************/

std::string segmentation_image_topic = "/smalldrop/rviz/segmentation_image";
camera_topics_t camera_topics;
unsigned int image_width = 640;
unsigned int image_height = 480;
bool use_otsu = false;
bool show_preview = false;

/**
 * Function prototypes
 *********************************************************************************************/

sensor_msgs::ImagePtr getSegmentationImageForPublication(const sensor_msgs::Image& img, img_wsp_calibration_t calibration_data);
bool processCmdArgs(int argc, char **argv);

/**
 * Main
 *********************************************************************************************/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detect_wound");
  ros::NodeHandle nh;

  if (!processCmdArgs(argc, argv)) return 0;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_segmentation = it.advertise(segmentation_image_topic, 1);

  double distance = 0.702;
  double wsp_w = 0.838 * distance; 
  double wsp_h = 0.75 * wsp_w;
  img_wsp_calibration_t calibration_data = {
    .img_width = image_width,
    .img_height = image_height,
    .wsp_x_min = -wsp_w/2,
    .wsp_x_max = wsp_w/2,
    .wsp_y_min = -wsp_h/2,
    .wsp_y_max = wsp_h/2
  };

  camera_topics.rgb_info_topic = "/smalldrop/vision/camera/color/camera_info";
  camera_topics.rgb_image_topic = "/smalldrop/vision/camera/color/image_raw";
  camera_topics.ir1_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir1_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.ir2_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir2_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.depth_info_topic = "/smalldrop/vision/camera/depth/camera_info";
  camera_topics.depth_image_topic = "/smalldrop/vision/camera/depth/image_raw";
  camera_topics.rgb_pcloud_topic = "/smalldrop/vision/camera/depth/color/points";

  CameraD415 cam(true, camera_topics);
  cam.turnOn();

  ros::Rate r(10); // 10 Hz
  while (ros::ok())
  {
    sensor_msgs::ImagePtr msg_segmentation = getSegmentationImageForPublication(cam.getRGBImage(), calibration_data);
    pub_segmentation.publish(msg_segmentation);

    ros::spinOnce();
    r.sleep();
  }

  cam.turnOff();
  
  return 0;
}

/**
 * Prepare segmentation area image to be published for rviz
 */
sensor_msgs::ImagePtr getSegmentationImageForPublication(const sensor_msgs::Image &img, img_wsp_calibration_t calibration_data)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat rsize(calibration_data.img_height, calibration_data.img_width, CV_8UC3);
  rsize = cv::Scalar::all(0);

  try
  {
    if (sensor_msgs::image_encodings::isColor(img.encoding))
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    else
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    // ROS_ERROR("cv_bridge exception: %s", e.what());
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rsize).toImageMsg();
    return msg;
  }

  try
  {
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
    int threshold_type = cv::THRESH_BINARY_INV;
    if (use_otsu) 
      threshold_type |= cv::THRESH_OTSU;

    int const max_BINARY_value = 255;
    cv::threshold(grey, bin, threshold_value, max_BINARY_value, threshold_type);

    // Get contours
    contours_t contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(rsize, contours, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    if (show_preview)
    {
      cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
      cv::imshow("Display window", rsize);                   // Show our image inside it.
      cv::waitKey(0);
    }
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Error creating segmentation image: %s", e.what());
  }

  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rsize).toImageMsg();
  return msg;
}

bool processCmdArgs(int argc, char **argv)
{
  // Process command-line arguments
  int opt;
  const char* const short_opts = ":os";

  while ((opt = getopt(argc, argv, short_opts)) != -1)
  {
    switch (opt)
    {
      case 'o':
        use_otsu = true;
        break;
      case 's':
        show_preview = true;
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "detect_wound -o -s" << std::endl; 
        std::cout << "o: use otsu binarization" << std::endl; 
        std::cout << "s: show preview window" << std::endl; 
        return false;
        break;
    }
  }
  return true;
}