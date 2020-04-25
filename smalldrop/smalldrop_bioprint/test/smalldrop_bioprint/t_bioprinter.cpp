// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_bioprinter.cpp
 * \brief Test file for Bioprinter class.
 */

#include <gtest/gtest.h>
#include <smalldrop_bioprint/bioprinter.h>
#include <smalldrop_bioprint/system_config.h>
#include <smalldrop_state/system_state.h>
#include <iostream>

using namespace smalldrop::smalldrop_bioprint;
using namespace smalldrop::smalldrop_state;

class BioprinterTest : public ::testing::Test
{
protected:
  ros::NodeHandle nh;
  // std::string catkin_ws;
  std::unique_ptr<SystemState> ss;
  std::unique_ptr<SystemConfig> config;

  void SetUp() override
  {
    std::unique_ptr<SystemState> ss_ptr(new SystemState());
    std::unique_ptr<SystemConfig> config_ptr(new SystemConfig(nh));
    ss = std::move(ss_ptr);
    config = std::move(config_ptr);
  }

  void TearDown() override
  {
  }

public:
  void wait(unsigned int delay)
  {
    // Wait for delay seconds
    int freq = 100;  // 100 Hz
    ros::Rate r(freq);
    int t = freq * delay;
    while (t-- > 0)
      r.sleep();
  }

  void waitSpin(unsigned int delay)
  {
    // Wait for delay seconds
    int freq = 100;  // 100 Hz
    ros::Rate r(freq);
    int t = freq * delay;
    while (t-- > 0)
    {
      ros::spinOnce();
      r.sleep();
    }
  }
};

TEST_F(BioprinterTest, isSimulationMode)
{
  Bioprinter bp1(std::move(ss), std::move(config), true, true); // is_simulation = true, is_dev = true;
  EXPECT_TRUE(bp1.isSimulation());  

  Bioprinter bp2(std::move(ss), std::move(config), false, true); // is_simulation = false, is_dev = true;
  EXPECT_FALSE(bp2.isSimulation());
}

TEST_F(BioprinterTest, isDevelopmentMode)
{
  Bioprinter bp1(std::move(ss), std::move(config), true, true); // is_simulation = true, is_dev = true;
  EXPECT_TRUE(bp1.isDevelopment());  

  Bioprinter bp2(std::move(ss), std::move(config), true, false); // is_simulation = true, is_dev = false;
  EXPECT_FALSE(bp2.isDevelopment());
}

TEST_F(BioprinterTest, isPublishingState)
{
  std::unique_ptr<SystemState> ss_(new SystemState());

  Bioprinter bp(std::move(ss), std::move(config), true, true); // is_simulation = true, is_dev = true;

  // Wait 3 seconds for the topics to update
  waitSpin(3);

  std::string state = ss_->getSystemState();

  EXPECT_EQ(state, "OFF");  
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_bioprinter");
  return RUN_ALL_TESTS();
}
