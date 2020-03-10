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
RemoteControllerMode::RemoteControllerMode(std::string name)
{
  mode_name_ = name;
}

/**
 * \copybrief RemoteControllerMode::RemoteControllerMode(std::string name, keymap_t keymap)
 */
RemoteControllerMode::RemoteControllerMode(std::string name, keymap_t keymap)
{
  mode_name_ = name;
  key_map_ = keymap;
}

/**
 * \copybrief IRemoteControllerMode::getKeyMap()
 */
keymap_t RemoteControllerMode::getKeyMap(void)
{
  return key_map_;
}

/**
 * \copybrief IRemoteControllerMode::getKeyAction()
 */
std::function<void(void)> RemoteControllerMode::getKeyAction(std::string key)
{
  return key_map_[key];
}

/**
 * \copybrief IRemoteControllerMode::getName()
 */
std::string RemoteControllerMode::getName(void)
{
  return mode_name_;
}

/**
 * \copybrief RemoteControllerMode::setKeyMap()
 */
void RemoteControllerMode::setKeyMap(keymap_t keymap)
{
  key_map_ = keymap;
}

} // namespace smalldrop_teleoperation

} // namespace smalldrop