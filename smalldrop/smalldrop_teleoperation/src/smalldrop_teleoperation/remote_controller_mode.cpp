// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file remote_controller_mode.cpp
 * \brief Implements class RemoteControllerMode that represents a remote controller mode.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
/**
 * \copybrief RemoteControllerMode::RemoteControllerMode(std::string name)
 */
RemoteControllerMode::RemoteControllerMode(std::string name) : mode_name_(name)
{
}

/**
 * \copybrief RemoteControllerMode::(std::string name, button_map_t button_map, action_map_t action_map)
 */
RemoteControllerMode::RemoteControllerMode(std::string name, button_map_t button_map, action_map_t action_map)
: mode_name_(name), button_map_(button_map), action_map_(action_map) 
{
}

/**
 * \copybrief IRemoteControllerMode::getButtonMap()
 */
button_map_t RemoteControllerMode::getButtonMap() const
{
  return button_map_;
}

/**
 * \copybrief IRemoteControllerMode::getActionMap()
 */
action_map_t RemoteControllerMode::getActionMap() const
{
  return action_map_;
}

/**
 * \copybrief IRemoteControllerMode::getButtonAction()
 */
std::function<bool(smalldrop_bioprint::SystemState*)> RemoteControllerMode::getButtonAction(std::string function)
{
  return action_map_[function];
}

/**
 * \copybrief IRemoteControllerMode::getName()
 */
std::string RemoteControllerMode::getName() const
{
  return mode_name_;
}

/**
 * \copybrief RemoteControllerMode::setKeyMaps()
 */
void RemoteControllerMode::setKeyMaps(button_map_t button_map, action_map_t action_map)
{
  button_map_ = button_map;
  action_map_ = action_map;
}

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop