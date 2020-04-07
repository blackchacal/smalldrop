// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_remote_controller_mode.cpp
 * \brief Test file for RemoteControllerMode class.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_state/system_state.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace smalldrop::smalldrop_teleoperation;
using namespace smalldrop::smalldrop_state;

class RemoteControllerModeTest : public ::testing::Test
{
  protected:
    action_map_t action_map1, action_map2;
    button_map_t button_map1, button_map2;

    void SetUp() override 
    {
      action_map1 = {
        {"action1", boost::bind(&RemoteControllerModeTest::action1, this, _1)},
        {"action2", boost::bind(&RemoteControllerModeTest::action2, this, _1)},
        {"action3", boost::bind(&RemoteControllerModeTest::action3, this, _1)},
      };
      action_map2 = {
        {"action1", boost::bind(&RemoteControllerModeTest::action1, this, _1)},
        {"action2", boost::bind(&RemoteControllerModeTest::action2, this, _1)},
        {"action4", boost::bind(&RemoteControllerModeTest::action3, this, _1)},
      };

      button_map1 = {
        {"action1", "bt1"},
        {"action2", "bt2"},
        {"action3", "bt3"},
      };
      button_map2 = {
        {"action1", "bt1"},
        {"action2", "bt2"},
        {"action4", "bt3"},
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
};

TEST_F(RemoteControllerModeTest, setsNameCorrectly)
{
  RemoteControllerMode mode1("mode1");
  EXPECT_EQ(mode1.getName(), "mode1");

  RemoteControllerMode mode2("mode2", button_map2, action_map2);
  EXPECT_EQ(mode2.getName(), "mode2");
}

TEST_F(RemoteControllerModeTest, setsAndGetsButtonMapCorrectly)
{
  RemoteControllerMode mode1("mode1", button_map1, action_map1);
  button_map_t stored_button_map1 = mode1.getButtonMap();
  button_map_t::iterator it1 = stored_button_map1.begin();
 
	// Iterate over the map using Iterator till end.
	while (it1 != stored_button_map1.end())
	{
		EXPECT_TRUE(button_map1.count(it1->first) > 0);
		it1++;
	}
  EXPECT_TRUE(stored_button_map1.size() == button_map1.size());

  RemoteControllerMode mode2("mode2");
  mode2.setKeyMaps(button_map1, action_map1);
  button_map_t stored_button_map2 = mode2.getButtonMap();
  button_map_t::iterator it2 = stored_button_map2.begin();
 
	// Iterate over the map using Iterator till end.
	while (it2 != stored_button_map2.end())
	{
		EXPECT_TRUE(button_map1.count(it2->first) > 0);
		it2++;
	}
  EXPECT_TRUE(stored_button_map2.size() == button_map1.size());
}

TEST_F(RemoteControllerModeTest, setsAndGetsActionMapCorrectly)
{
  RemoteControllerMode mode1("mode1", button_map1, action_map1);
  action_map_t stored_action_map1 = mode1.getActionMap();
  action_map_t::iterator it1 = stored_action_map1.begin();
 
	// Iterate over the map using Iterator till end.
	while (it1 != stored_action_map1.end())
	{
		EXPECT_TRUE(action_map1.count(it1->first) > 0);
		it1++;
	}
  EXPECT_TRUE(stored_action_map1.size() == action_map1.size());

  RemoteControllerMode mode2("mode2");
  mode2.setKeyMaps(button_map1, action_map1);
  action_map_t stored_action_map2 = mode2.getActionMap();
  action_map_t::iterator it2 = stored_action_map2.begin();
 
	// Iterate over the map using Iterator till end.
	while (it2 != stored_action_map2.end())
	{
		EXPECT_TRUE(action_map1.count(it2->first) > 0);
		it2++;
	}
  EXPECT_TRUE(stored_action_map2.size() == action_map1.size());
}

TEST_F(RemoteControllerModeTest, returnsRightFunction)
{
  SystemState ss;

  RemoteControllerMode mode1("mode1", button_map2, action_map2);
  std::function<bool(smalldrop::smalldrop_state::SystemState*)> f = mode1.getButtonAction("action1");
  EXPECT_TRUE(f(&ss));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "t_remote_controller_mode");
  return RUN_ALL_TESTS();
}
