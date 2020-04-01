// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_bioprinter.cpp
 * \brief Test file for Bioprinter class.
 */

#include <gtest/gtest.h>
#include <smalldrop_bioprint/bioprinter.h>
#include <smalldrop_bioprint/system_state.h>
#include <iostream>

using namespace smalldrop::smalldrop_bioprint;

class SpaceMouseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
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

TEST_F(SpaceMouseTest, isSimulationMode)
{
  Bioprinter bp1(true, true); // is_simulation = true, is_dev = true;
  EXPECT_TRUE(bp1.isSimulation());  

  Bioprinter bp2(false, true); // is_simulation = false, is_dev = true;
  EXPECT_FALSE(bp2.isSimulation());
}

TEST_F(SpaceMouseTest, isDevelopmentMode)
{
  Bioprinter bp1(true, true); // is_simulation = true, is_dev = true;
  EXPECT_TRUE(bp1.isDevelopment());  

  Bioprinter bp2(true, false); // is_simulation = true, is_dev = false;
  EXPECT_FALSE(bp2.isDevelopment());
}

TEST_F(SpaceMouseTest, isPublishingState)
{
  SystemState ss;
  Bioprinter bp(true, true); // is_simulation = true, is_dev = true;

  // Wait 3 seconds for the topics to update
  waitSpin(3);

  std::string state = ss.getSystemState();

  EXPECT_EQ(state, "OFF");  
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_bioprinter");
  return RUN_ALL_TESTS();
}
