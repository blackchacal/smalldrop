// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file remote_controller_mode.h
 * \brief Defines class that represents a remote controller mode.
 */

#ifndef _SMALLDROP_REMOTE_CONTROLLER_MODE_H
#define _SMALLDROP_REMOTE_CONTROLLER_MODE_H

#include <functional>
#include <map>

#include <smalldrop_teleoperation/i_remote_controller_mode.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
/**
 * \class RemoteControllerMode
 * \brief Class that represents a remote controller mode for teleoperation.
 */
class RemoteControllerMode : public IRemoteControllerMode
{
private:
  /**
   * Class members
   *****************************************************************************************/

  std::string mode_name_;   /** \var Mode name. */
  button_map_t button_map_; /** Map between remote controller function and buttons. */
  action_map_t action_map_; /** Map between remote controller function and actions. */

public:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \brief Default constructor.
   *
   * \param name Mode name.
   */
  RemoteControllerMode(std::string name);

  /**
   * \brief Overloaded constructor that receives a key map.
   *
   * \param name Mode name.
   * \param button_map Dictionary matching the function name and button names.
   * \param action_map Dictionary matching the function name and button actions.
   */
  RemoteControllerMode(std::string name, button_map_t button_map, action_map_t action_map);

  /**
   * \copydoc IRemoteControllerMode::getButtonMap()
   */
  virtual button_map_t getButtonMap() const override;

  /**
   * \copydoc IRemoteControllerMode::getActionMap()
   */
  virtual action_map_t getActionMap() const override;

  /**
   * \copydoc IRemoteControllerMode::getButtonAction()
   */
  virtual std::function<bool(smalldrop_bioprint::SystemState*)> getButtonAction(std::string function) override;

  /**
   * \copydoc IRemoteControllerMode::getName()
   */
  virtual std::string getName() const override;

  /**
   * \fn void setKeyMaps(button_map_t button_map, action_map_t action_map)
   * \brief Sets the button and action maps used on the mode.
   *
   * \param button_map Dictionary matching the function name and button names.
   * \param action_map Dictionary matching the function name and button actions.
   */
  void setKeyMaps(button_map_t button_map, action_map_t action_map);
};

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop

#endif  //_SMALLDROP_REMOTE_CONTROLLER_MODE_H