// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file i_remote_controller_mode.h
 * \brief Defines IRemoteControllerMode interface for remote controller modes.
 */

#ifndef _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_MODE_H
#define _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_MODE_H

#include <smalldrop_bioprint/system_state.h>

#include <functional>
#include <map>
#include <string>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
/**
 * \typedef
 * \brief Typedef for callback functions.
 */
typedef std::function<bool(smalldrop_bioprint::SystemState*)> callback_t;

/**
 * \typedef
 * \brief Typedef for map of controller function names and associated actions.
 */
typedef std::map<std::string, callback_t> action_map_t;

/**
 * \typedef
 * \brief Typedef for map of controller function names and button names.
 */
typedef std::map<std::string, std::string> button_map_t;

/**
 * \class IRemoteControllerMode
 * \brief Interface for remote controller modes for teleoperation.
 */
class IRemoteControllerMode
{
public:
  virtual ~IRemoteControllerMode()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn virtual button_map_t getButtonMap() const
   * \brief Returns the button map used on the mode.
   */
  virtual button_map_t getButtonMap() const = 0;

  /**
   * \fn virtual action_map_t getActionMap() const
   * \brief Returns the action map used on the mode.
   */
  virtual action_map_t getActionMap() const = 0;

  /**
   * \fn virtual std::function<bool(smalldrop_bioprint::SystemState*)> getButtonAction(std::string function)
   * \brief Returns the action function associated with a specified key.
   *
   * \param function Function name for which the action will be returned.
   */
  virtual std::function<bool(smalldrop_bioprint::SystemState*)> getButtonAction(std::string function) = 0;

  /**
   * \fn virtual std::string getName() const
   * \brief Returns the mode name.
   */
  virtual std::string getName() const = 0;
};

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop

#endif  // _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_MODE_H