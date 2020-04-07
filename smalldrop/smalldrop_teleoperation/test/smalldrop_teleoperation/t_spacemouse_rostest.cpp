// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_spacemouse_rostest.cpp
 * \brief Test file for SpaceMouse functions dependent on robot arm nodes.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_teleoperation/spacemouse.h>
#include <smalldrop_teleoperation/teleoperation_actions.h>
#include <smalldrop_state/system_state.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace smalldrop::smalldrop_teleoperation;
using namespace smalldrop::smalldrop_state;

class SpaceMouseTest : public ::testing::Test
{
  protected:
    std::string topic1, topic2, topic3, topic4;
    action_map_t action_map1, action_map2, action_map3;
    button_map_t button_map1, button_map2, button_map3;

    void SetUp() override 
    {
      topic1 = "/spacenav/joy";
      topic2 = "/smalldrop/smalldrop_teleoperation/topic2";
      topic3 = "/smalldrop/smalldrop_teleoperation/topic3";
      topic4 = "/smalldrop/smalldrop_teleoperation/topic4";

      action_map1 = {
        {"action1", boost::bind(&SpaceMouseTest::action1, this, _1)},
        {"action2", boost::bind(&SpaceMouseTest::action2, this, _1)},
        {"action3", boost::bind(&SpaceMouseTest::action3, this, _1)}
      };

      action_map2 = {
        {"mode", boost::bind(&SpaceMouseTest::mode, this, _1)},
        {"joy", boost::bind(&SpaceMouseTest::joy, this, _1)},
        {"ok", boost::bind(&SpaceMouseTest::ok, this, _1)}
      };

      action_map3 = {
        {"joy", boost::bind(&teleop_actions::moveRobotArm, _1)},
        {"mode", boost::bind(&teleop_actions::changeMode, _1)},
        {"ok", boost::bind(&teleop_actions::publishSegmentationPoint, _1)}
      };

      button_map1 = {
        {"action1", "bt1"},
        {"action2", "bt2"},
        {"action3", "bt3"},
      };
      button_map2 = {
        {"mode", "buttons_1"},
        {"joy", "axes_*"},
        {"ok", "buttons_0"},
      };
      button_map3 = {
        {"mode", "buttons_0"},
        {"joy", "axes_*"},
        {"ok", "buttons_1"},
      };
    }

    void TearDown() override {}

  public:
    bool action1(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "action1" << std::endl;
      return true;
    }

    bool action2(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "action2" << std::endl;
      return true;
    }

    bool action3(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "action3" << std::endl;
      return true;
    }

    bool mode(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "change mode" << std::endl;
      return true;
    }

    bool joy(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "change joy" << std::endl;
      return true;
    }

    bool ok(smalldrop::smalldrop_state::SystemState* system_state)
    {
      std::cout << "change ok" << std::endl;
      return true;
    }

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

TEST_F(SpaceMouseTest, connectController)
{
  SystemState ss;

  SpaceMouse sm(topic1, &ss);
  EXPECT_TRUE(sm.turnOn());

  // Wait for 1 second
  wait(1);

  sm.turnOff();
}

TEST_F(SpaceMouseTest, disconnectController)
{
  SystemState ss;

  SpaceMouse sm(topic1, &ss);
  sm.turnOn();

  // Wait for 1 second
  wait(1);

  ASSERT_TRUE(sm.turnOff());
}

TEST_F(SpaceMouseTest, stateIsUpdated)
{
  SystemState ss;

  SpaceMouse sm(topic1, &ss);
  sm.turnOn();

  // Wait for 2 seconds for the topic to be publish and update the state
  waitSpin(2);

  sensor_msgs::Joy& state = sm.getState();
  
  EXPECT_TRUE(state.axes[0] != -1);
  EXPECT_TRUE(state.axes[1] != -1);
  EXPECT_TRUE(state.axes[2] != -1);
  EXPECT_TRUE(state.axes[3] != -1);
  EXPECT_TRUE(state.axes[4] != -1);
  EXPECT_TRUE(state.axes[5] != -1);
  EXPECT_TRUE(state.buttons[0] != -1);
  EXPECT_TRUE(state.buttons[1] != -1);

  sm.turnOff();
}

TEST_F(SpaceMouseTest, callButtonAction)
{
  SystemState ss;

  // Setup modes
  IRemoteControllerMode* mode = new RemoteControllerMode("default", button_map2, action_map2);
  std::list<IRemoteControllerMode*> modes;
  modes.push_front(mode);

  testing::internal::CaptureStdout(); // Capture std::cout during test

  SpaceMouse sm(topic1, &ss);
  sm.configModes(modes);
  sm.turnOn();

  // Wait for 1 second
  wait(1);

  sm.turnOff();

  std::string output = testing::internal::GetCapturedStdout();

  // Always calls joy two times before stabilizing
  EXPECT_GE(output.find("change joy"), (unsigned int)0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_spacemouse_rostest");
  return RUN_ALL_TESTS();
}
