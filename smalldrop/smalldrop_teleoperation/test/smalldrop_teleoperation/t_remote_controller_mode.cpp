// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_remote_controller_mode.cpp
 * \brief Test file for RemoteControllerMode class.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <gtest/gtest.h>
#include <iostream>

using namespace smalldrop::smalldrop_teleoperation;

class RemoteControllerModeTest : public ::testing::Test
{
  protected:
    keymap_t keymap1, keymap2;

    void SetUp() override 
    {
      keymap1 = {
        {"action1", std::bind(&RemoteControllerModeTest::action1, this)},
        {"action2", std::bind(&RemoteControllerModeTest::action2, this)},
        {"action3", std::bind(&RemoteControllerModeTest::action3, this)},
      };
      keymap2 = {
        {"action1", std::bind(&RemoteControllerModeTest::action1, this)},
        {"action2", std::bind(&RemoteControllerModeTest::action2, this)},
        {"action4", std::bind(&RemoteControllerModeTest::action3, this)},
      };
    }

    void TearDown() override {}

  public:
    void action1(void)
    {
      std::cout << "action1" << std::endl;
    }

    void action2(void)
    {
      std::cout << "action2" << std::endl;
    }

    void action3(void)
    {
      std::cout << "action3" << std::endl;
    }
};

TEST_F(RemoteControllerModeTest, setsNameCorrectly)
{
  RemoteControllerMode mode1("mode1");
  EXPECT_EQ(mode1.getName(), "mode1");

  RemoteControllerMode mode2("mode2", keymap2);
  EXPECT_EQ(mode2.getName(), "mode2");
}

TEST_F(RemoteControllerModeTest, setsAndGetsKeymapCorrectly)
{
  RemoteControllerMode mode1("mode1", keymap1);
  keymap_t stored_keymap1 = mode1.getKeyMap();
  keymap_t::iterator it1 = stored_keymap1.begin();
 
	// Iterate over the map using Iterator till end.
	while (it1 != stored_keymap1.end())
	{
		EXPECT_TRUE(keymap1.count(it1->first) > 0);
		it1++;
	}
  EXPECT_TRUE(stored_keymap1.size() == keymap1.size());


  RemoteControllerMode mode2("mode2");
  mode2.setKeyMap(keymap1);
  keymap_t stored_keymap2 = mode2.getKeyMap();
  keymap_t::iterator it2 = stored_keymap2.begin();
 
	// Iterate over the map using Iterator till end.
	while (it2 != stored_keymap2.end())
	{
		EXPECT_TRUE(keymap1.count(it2->first) > 0);
		it2++;
	}
  EXPECT_TRUE(stored_keymap2.size() == keymap1.size());
}

TEST_F(RemoteControllerModeTest, returnsRightFunction)
{
  RemoteControllerMode mode1("mode1", keymap2);
  std::function<void()> f = mode1.getKeyAction("action1");
  f();
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
