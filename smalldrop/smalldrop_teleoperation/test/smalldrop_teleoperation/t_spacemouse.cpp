// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_spacemouse.cpp
 * \brief Test file for SpaceMouse class.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_teleoperation/spacemouse.h>
#include <smalldrop_teleoperation/teleoperation_actions.h>
#include <smalldrop_bioprint/system_state.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace smalldrop::smalldrop_teleoperation;
using namespace smalldrop::smalldrop_bioprint;

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
    bool action1(smalldrop::smalldrop_bioprint::SystemState* system_state)
    {
      std::cout << "action1" << std::endl;
      return true;
    }

    bool action2(smalldrop::smalldrop_bioprint::SystemState* system_state)
    {
      std::cout << "action2" << std::endl;
      return true;
    }

    bool action3(smalldrop::smalldrop_bioprint::SystemState* system_state)
    {
      std::cout << "action3" << std::endl;
      return true;
    }

    bool mode(smalldrop::smalldrop_bioprint::SystemState* system_state)
    {
      std::cout << "change mode" << std::endl;
      return true;
    }

    bool joy(smalldrop::smalldrop_bioprint::SystemState* system_state)
    {
      std::cout << "change joy" << std::endl;
      return true;
    }

    bool ok(smalldrop::smalldrop_bioprint::SystemState* system_state)
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

TEST_F(SpaceMouseTest, setStateTopic)
{
  SystemState ss;

  SpaceMouse sm1(topic1, &ss);
  SpaceMouse sm2(topic2, &ss);
  SpaceMouse sm3(topic3, &ss);
  SpaceMouse sm4(topic4, &ss);

  ASSERT_EQ(sm1.getStateTopic(), topic1);
  ASSERT_EQ(sm2.getStateTopic(), topic2);
  ASSERT_EQ(sm3.getStateTopic(), topic3);
  ASSERT_EQ(sm4.getStateTopic(), topic4);
}

TEST_F(SpaceMouseTest, setKeyModes)
{
  SystemState ss;

  RemoteControllerMode mode1("default", button_map1, action_map1);
  std::list<IRemoteControllerMode*> modes;
  modes.push_front(&mode1);

  SpaceMouse sm(topic1, modes, &ss);

  EXPECT_EQ(sm.whatMode()->getName(), mode1.getName());
}

TEST_F(SpaceMouseTest, changeKeyModes)
{
  SystemState ss;

  RemoteControllerMode mode1("default", button_map1, action_map1);
  RemoteControllerMode mode2("mode2", button_map2, action_map2);
  std::list<IRemoteControllerMode*> modes;
  modes.push_back(&mode1);
  modes.push_back(&mode2);

  SpaceMouse sm(topic1, modes, &ss);

  EXPECT_EQ(sm.whatMode()->getName(), mode1.getName());
  sm.changeMode();
  EXPECT_EQ(sm.whatMode()->getName(), mode2.getName());
  sm.changeMode();
  EXPECT_EQ(sm.whatMode()->getName(), mode1.getName());
}

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

TEST_F(SpaceMouseTest, changeMode)
{
  SystemState ss;
  waitSpin(2);

  // Setup modes
  RemoteControllerMode mode1("mode1", button_map3, action_map3);
  RemoteControllerMode mode2("mode2", button_map2, action_map2);
  std::list<IRemoteControllerMode*> modes;
  modes.push_back(&mode1);
  modes.push_back(&mode2);

  SpaceMouse sm(topic1, modes, &ss);
  sm.turnOn();

  IRemoteControllerMode* start_mode = sm.whatMode();

  // Wait for 10 second
  waitSpin(10);

  IRemoteControllerMode* changed_mode = sm.whatMode();

  // IMPORTANT: Need to press the mode button from the first mode (button 0)
  EXPECT_TRUE(start_mode->getName().compare(changed_mode->getName()) != 0);

  sm.turnOff();
}

TEST_F(SpaceMouseTest, callButtonAction)
{
  SystemState ss;

  // Setup modes
  RemoteControllerMode mode("default", button_map2, action_map2);
  std::list<IRemoteControllerMode*> modes;
  modes.push_front(&mode);

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

TEST_F(SpaceMouseTest, callMoveRobotArmTeleoperationAction)
{
  SystemState ss;
  waitSpin(2);

  // Setup modes
  RemoteControllerMode mode("default", button_map3, action_map3);
  std::list<IRemoteControllerMode*> modes;
  modes.push_back(&mode);

  SpaceMouse sm(topic1, modes, &ss);
  sm.turnOn();

  geometry_msgs::Pose initial_pose = ss.getRobotArmPose();

  // Wait for 10 second
  waitSpin(10);

  geometry_msgs::Pose final_pose = ss.getRobotArmPose();

  // IMPORTANT: Need to move the joystick during the test to change robot pose
  EXPECT_TRUE(initial_pose.position.x != final_pose.position.x);
  EXPECT_TRUE(initial_pose.position.y != final_pose.position.y);
  EXPECT_TRUE(initial_pose.position.z != final_pose.position.z);
  EXPECT_TRUE(initial_pose.orientation.x != final_pose.orientation.x);
  EXPECT_TRUE(initial_pose.orientation.y != final_pose.orientation.y);
  EXPECT_TRUE(initial_pose.orientation.z != final_pose.orientation.z);
  EXPECT_TRUE(initial_pose.orientation.w != final_pose.orientation.w);

  sm.turnOff();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_spacemouse");
  return RUN_ALL_TESTS();
}
