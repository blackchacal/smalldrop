// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file remote_controller.cpp
 * \brief Defines class for remote controllers that implements the IRemoteController interface.
 */

#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_teleoperation/remote_controller.h>
#include <iostream>

namespace smalldrop
{
namespace smalldrop_teleoperation
{

/*****************************************************************************************
 * Public methods
 *****************************************************************************************/

/**
 * \copybrief RemoteController::RemoteController()
 */
RemoteController::RemoteController() : connected_(false), active_mode_(nullptr)
{
  mode_it_ = modes_.begin();
}

/**
 * \copybrief RemoteController::RemoteController(std::list<IRemoteControllerMode*> modes)
 */
RemoteController::RemoteController(std::list<IRemoteControllerMode*> modes) : connected_(false), modes_(modes)
{
  mode_it_ = modes_.begin();
  active_mode_ = modes.front();
  setupButtons();
}

/**
 * \copydoc IRemoteController::turnOn()
 */
bool RemoteController::turnOn()
{
  connect();
  return isConnected();
}

/**
 * \copydoc IRemoteController::turnOff()
 */
bool RemoteController::turnOff()
{
  disconnect();
  return !isConnected();
}

/**
 * \copydoc IRemoteController::isConnected()
 */
bool RemoteController::isConnected() const
{
  return connected_;
}

/**
 * \copydoc IRemoteController::configModes()
 */
bool RemoteController::configModes(std::list<IRemoteControllerMode*> modes)
{
  modes_ = modes;
  active_mode_ = modes.front();
  mode_it_ = modes_.begin();
  setupButtons();
  return true;
}

/**
 * \copybrief RemoteController::whatMode()
 */
IRemoteControllerMode* RemoteController::whatMode() const
{
  return active_mode_;
}

/**
 * \copybrief RemoteController::isMode()
 */
bool RemoteController::isMode(std::string mode_name) const
{
  return active_mode_->getName().compare(mode_name) == 0;
}

/**
 * \copybrief RemoteController::changeMode()
 */
void RemoteController::changeMode()
{
  // Update the modes list iterator

  // The std::list::end function returns the iterator to the position after the last
  // element. Since we want to compare if it points to the last element, just compare
  // against a one position decremented iterator, from the end.
  if (mode_it_ != (--modes_.end()))
    mode_it_++;
  else
    mode_it_ = modes_.begin();

  active_mode_ = *mode_it_;

  setupButtons();
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \copybrief RemoteController::isButtonSet()
 */
bool RemoteController::isButtonSet(std::string button) const
{
  if (button.compare("on_off") == 0)
    return isOnOffButtonSet();
  else if (button.compare("mode") == 0)
    return isModeButtonSet();
  else if (button.compare("ok") == 0)
    return isOkButtonSet();
  else if (button.compare("cancel") == 0)
    return isCancelButtonSet();
  else if (button.compare("abort") == 0)
    return isAbortButtonSet();
  else if (button.compare("up") == 0)
    return isUpButtonSet();
  else if (button.compare("down") == 0)
    return isDownButtonSet();
  else if (button.compare("left") == 0)
    return isLeftButtonSet();
  else if (button.compare("right") == 0)
    return isRightButtonSet();
  else if (button.compare("joy") == 0)
    return isJoyButtonSet();
  else
    return false;
}

/**
 * \copybrief RemoteController::callButtonAction()
 */
bool RemoteController::callButtonAction(std::string function, smalldrop_state::SystemState* system_state)
{
  return active_mode_->getButtonAction(function)(system_state); // Call the action
}

/*****************************************************************************************
 * Private methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copydoc RemoteController::setupButtons()
 */
void RemoteController::setupButtons()
{
  button_map_t button_map = active_mode_->getButtonMap();

  on_off_button_ = (button_map.count("on_off")) ? button_map.find("on_off")->second : "";
  mode_button_ = (button_map.count("mode")) ? button_map.find("mode")->second : "";
  ok_button_ = (button_map.count("ok")) ? button_map.find("ok")->second : "";
  cancel_button_ = (button_map.count("cancel")) ? button_map.find("cancel")->second : "";
  abort_button_ = (button_map.count("abort")) ? button_map.find("abort")->second : "";
  up_button_ = (button_map.count("up")) ? button_map.find("up")->second : "";
  down_button_ = (button_map.count("down")) ? button_map.find("down")->second : "";
  left_button_ = (button_map.count("left")) ? button_map.find("left")->second : "";
  right_button_ = (button_map.count("right")) ? button_map.find("right")->second : "";
  joy_button_ = (button_map.count("joy")) ? button_map.find("joy")->second : "";
}

/**
 * \copybrief RemoteController::isOnOffButtonSet()
 */
bool RemoteController::isOnOffButtonSet() const
{
  return !on_off_button_.empty();
}

/**
 * \copybrief RemoteController::isModeButtonSet()
 */
bool RemoteController::isModeButtonSet() const
{
  return !mode_button_.empty();
}

/**
 * \copybrief RemoteController::isOkButtonSet()
 */
bool RemoteController::isOkButtonSet() const
{
  return !ok_button_.empty();
}

/**
 * \copybrief RemoteController::isCancelButtonSet()
 */
bool RemoteController::isCancelButtonSet() const
{
  return !cancel_button_.empty();
}

/**
 * \copybrief RemoteController::isAbortButtonSet()
 */
bool RemoteController::isAbortButtonSet() const
{
  return !abort_button_.empty();
}

/**
 * \copybrief RemoteController::isUpButtonSet()
 */
bool RemoteController::isUpButtonSet() const
{
  return !up_button_.empty();
}

/**
 * \copybrief RemoteController::isDownButtonSet()
 */
bool RemoteController::isDownButtonSet() const
{
  return !down_button_.empty();
}

/**
 * \copybrief RemoteController::isLeftButtonSet()
 */
bool RemoteController::isLeftButtonSet() const
{
  return !left_button_.empty();
}

/**
 * \copybrief RemoteController::isRightButtonSet()
 */
bool RemoteController::isRightButtonSet() const
{
  return !right_button_.empty();
}

/**
 * \copybrief RemoteController::isJoyButtonSet()
 */
bool RemoteController::isJoyButtonSet() const
{
  return !joy_button_.empty();
}

} // namespace smalldrop_teleoperation

} // namespace smalldrop