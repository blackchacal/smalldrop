// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file smalldrop_teleoperation.cpp
 * \brief ROS node to initialise the teleoperation functionality.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_teleoperation/spacemouse.h>
#include <smalldrop_teleoperation/teleoperation_actions.h>
#include <smalldrop_bioprint/system_state.h>

using namespace smalldrop::smalldrop_teleoperation;
using namespace smalldrop::smalldrop_bioprint;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "default_teleoperation");

  SystemState ss;
  action_map_t action_map;
  button_map_t button_map;
  std::string topic = "/spacenav/joy";

  action_map = {
    {"joy", boost::bind(&teleop_actions::moveRobotArm, _1)},
    {"mode", boost::bind(&teleop_actions::changeMode, _1)},
    {"ok", boost::bind(&teleop_actions::publishSegmentationPoint, _1)}
  };
  button_map = {
    {"joy", "axes_*"},
    {"mode", "buttons_0"},
    {"ok", "buttons_1"},
  };

  // Setup modes
  RemoteControllerMode mode("default", button_map, action_map);
  std::list<IRemoteControllerMode*> modes = {&mode};

  SpaceMouse sm(topic, modes, &ss);
  sm.turnOn();

  int freq = 100; // 100 Hz
  ros::Rate r(freq); 
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  sm.turnOff();
  
  return 0;
}
