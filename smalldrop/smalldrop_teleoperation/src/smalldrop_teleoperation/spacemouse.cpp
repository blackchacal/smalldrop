// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file spacemouse.cpp
 * \brief Defines class for space mouse remote controller.
 */

#include <smalldrop_teleoperation/spacemouse.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief SpaceMouse::SpaceMouse(std::string topic, smalldrop_state::SystemState* system_state)
 */
SpaceMouse::SpaceMouse(std::string topic, smalldrop_state::SystemState* system_state)
  : RemoteController(), system_state_(system_state), state_topic_(topic)
{
  subscribeToStateTopic();

  state_.axes.reserve(6);
  state_.buttons.reserve(2);
  state_prev_.axes.reserve(6);
  state_prev_.buttons.reserve(2);

  state_.axes[0] = -1;
  state_.axes[1] = -1;
  state_.axes[2] = -1;
  state_.axes[3] = -1;
  state_.axes[4] = -1;
  state_.axes[5] = -1;
  state_.buttons[0] = -1;
  state_.buttons[1] = -1;

  state_prev_.axes[0] = -1;
  state_prev_.axes[1] = -1;
  state_prev_.axes[2] = -1;
  state_prev_.axes[3] = -1;
  state_prev_.axes[4] = -1;
  state_prev_.axes[5] = -1;
  state_prev_.buttons[0] = -1;
  state_prev_.buttons[1] = -1;
}

/**
 * \copybrief SpaceMouse::SpaceMouse(std::string topic, std::list<IRemoteControllerMode *> modes, smalldrop_state::SystemState* system_state)
 */
SpaceMouse::SpaceMouse(std::string topic, std::list<IRemoteControllerMode *> modes, smalldrop_state::SystemState* system_state)
  : RemoteController(modes), system_state_(system_state), state_topic_(topic)
{
  subscribeToStateTopic();

  state_.axes.reserve(6);
  state_.buttons.reserve(2);
  state_prev_.axes.reserve(6);
  state_prev_.buttons.reserve(2);

  state_.axes[0] = -1;
  state_.axes[1] = -1;
  state_.axes[2] = -1;
  state_.axes[3] = -1;
  state_.axes[4] = -1;
  state_.axes[5] = -1;
  state_.buttons[0] = -1;
  state_.buttons[1] = -1;

  state_prev_.axes[0] = -1;
  state_prev_.axes[1] = -1;
  state_prev_.axes[2] = -1;
  state_prev_.axes[3] = -1;
  state_prev_.axes[4] = -1;
  state_prev_.axes[5] = -1;
  state_prev_.buttons[0] = -1;
  state_prev_.buttons[1] = -1;
}

/**
 * \copybrief SpaceMouse::getStateTopic()
 */
std::string SpaceMouse::getStateTopic() const
{
  return state_topic_;
}

/**
 * \copybrief SpaceMouse::getState()
 */
sensor_msgs::Joy& SpaceMouse::getState()
{
  return state_;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief RemoteController::connect()
 */
void SpaceMouse::connect()
{
  bool spacenav_is_on = false;

  std::vector<std::string> nodes;
  ros::master::getNodes(nodes);

  for (unsigned int i = 0; i < nodes.size(); i++)
  {
    if (nodes[i].compare("/spacenav") == 0)
    {
      spacenav_is_on = true;
      break;
    }
  }

  if (spacenav_is_on)
    connected_ = true;
}

/**
 * \copybrief RemoteController::disconnect()
 */
void SpaceMouse::disconnect()
{
  connected_ = false;
}

/**
 * \copybrief RemoteController::subscribeToStateTopic()
 */
void SpaceMouse::subscribeToStateTopic()
{
  state_sub_ = nh_.subscribe<sensor_msgs::Joy>(state_topic_, 10, &SpaceMouse::getStateFromTopic, this);
}

/**
 * \copybrief RemoteController::getStateFromTopic()
 */
void SpaceMouse::getStateFromTopic(const sensor_msgs::Joy::ConstPtr &msg)
{
  if (connected_)
  {
    state_.axes[0] = msg->axes[0];
    state_.axes[1] = msg->axes[1];
    state_.axes[2] = msg->axes[2];
    state_.axes[3] = msg->axes[3];
    state_.axes[4] = msg->axes[4];
    state_.axes[5] = msg->axes[5];
    state_.buttons[0] = msg->buttons[0];
    state_.buttons[1] = msg->buttons[1];

    // Check if any button was pressed and call the associated action
    checkButtonPress();

    // Update previous data
    state_prev_.axes[0] = state_.axes[0];
    state_prev_.axes[1] = state_.axes[1];
    state_prev_.axes[2] = state_.axes[2];
    state_prev_.axes[3] = state_.axes[3];
    state_prev_.axes[4] = state_.axes[4];
    state_prev_.axes[5] = state_.axes[5];
    state_prev_.buttons[0] = state_.buttons[0];
    state_prev_.buttons[1] = state_.buttons[1];
  }
}

/**
 * \copybrief SpaceMouse::checkButtonPress()
 */
void SpaceMouse::checkButtonPress()
{
  if (active_mode_ != nullptr)
  {
    button_map_t button_map = active_mode_->getButtonMap();
    button_map_t::iterator it = button_map.begin();

    // Check control buttons
    if (state_.buttons[0] != state_prev_.buttons[0] && state_prev_.buttons[0] != -1 && !state_.buttons[0])
    {
      while (it != button_map.end())
      {
        if (it->second.compare("buttons_0") == 0 && isButtonSet(it->first))
        {
          if (it->first.compare("mode") == 0)
          {
            changeMode();
            callButtonAction(it->first, system_state_);
          }
          else
            callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.buttons[1] != state_prev_.buttons[1] && state_prev_.buttons[1] != -1 && !state_.buttons[1])
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if (it->second.compare("buttons_1") == 0 && isButtonSet(it->first))
        {
          if (it->first.compare("mode") == 0)
          {
            changeMode();
            callButtonAction(it->first, system_state_);
          }
          else
            callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }

    // Check joy buttons/positions
    if (state_.axes[0] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_x") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.axes[1] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_y") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.axes[2] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_z") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.axes[3] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_rx") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.axes[4] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_ry") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
    if (state_.axes[5] != 0)
    {
      it = button_map.begin();
      while (it != button_map.end())
      {
        if ((it->second.compare("axes_rz") == 0 || it->second.compare("axes_*") == 0) && isButtonSet(it->first))
        {
          callButtonAction(it->first, system_state_);
          break;
        }
        it++;
      }
    }
  }
}

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop