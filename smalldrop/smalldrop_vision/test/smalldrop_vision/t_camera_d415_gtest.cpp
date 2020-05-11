// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_camera_d415_gtest.cpp
 * \brief Test file for CameraD415 class.
 */

#include <smalldrop_vision/camera_d415.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace smalldrop::smalldrop_vision;

class CameraD415Test : public ::testing::Test
{
  protected:
    camera_topics_t topics;

    void SetUp() override 
    {
      topics.rgb_info_topic = "/realsense_plugin/camera/color/camera_info";
      topics.rgb_image_topic = "/realsense_plugin/camera/color/image_raw";
      topics.ir1_info_topic = "/realsense_plugin/camera/ir/camera_info";
      topics.ir1_image_topic = "/realsense_plugin/camera/ir/image_raw";
      topics.ir2_info_topic = "/realsense_plugin/camera/ir2/camera_info";
      topics.ir2_image_topic = "/realsense_plugin/camera/ir2/image_raw";
      topics.depth_info_topic = "/realsense_plugin/camera/depth/camera_info";
      topics.depth_image_topic = "/realsense_plugin/camera/depth/image_raw";
      topics.rgb_pcloud_topic = "/realsense_plugin/camera/point_cloud/data";
    }

    void TearDown() override {}

  public:
    void wait(unsigned int delay)
    {
      // Wait for delay seconds
      int freq = 100; // 100 Hz
      ros::Rate r(freq); 
      int t = freq * delay;
      while (t-- > 0)
        r.sleep();
    }

    void waitSpin(unsigned int delay)
    {
      // Wait for delay seconds
      int freq = 100; // 100 Hz
      ros::Rate r(freq); 
      int t = freq * delay;
      while (t-- > 0)
      {
        ros::spinOnce();
        r.sleep();
      }
    }
};

TEST_F(CameraD415Test, getTopicData)
{
  CameraD415 cam(true, topics);
  cam.turnOn();

  // Wait for 5 seconds
  waitSpin(5);

  std::cout << cam.getRGBInfo() << std::endl;
  // std::cout << cam.getRGBImage() << std::endl;
  // std::cout << cam.getIR1Info() << std::endl;
  // std::cout << cam.getIR1Image() << std::endl;
  // std::cout << cam.getIR2Info() << std::endl;
  // std::cout << cam.getIR2Image() << std::endl;
  // std::cout << cam.getDepthInfo() << std::endl;
  // std::cout << cam.getDepthImage() << std::endl;

  cam.turnOff();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_camera_d415_gtest");
  return RUN_ALL_TESTS();
}
