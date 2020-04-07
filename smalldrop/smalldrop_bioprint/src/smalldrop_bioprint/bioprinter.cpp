// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file bioprinter.cpp
 * \brief Defines a class that controls the whole smalldrop system. The public API should allow
 * the system to be controlled by a GUI.
 */

#include <smalldrop_bioprint/bioprinter.h>
#include <smalldrop_state/system_state.h>
#include <smalldrop_teleoperation/remote_controller_mode.h>
#include <smalldrop_teleoperation/spacemouse.h>
#include <smalldrop_teleoperation/teleoperation_actions.h>

#include <sstream>

namespace smalldrop
{
namespace smalldrop_bioprint
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief Bioprinter::Bioprinter(std::unique_ptr<SystemState> ss_ptr, const bool simulation, const bool development)
 */
Bioprinter::Bioprinter(std::unique_ptr<SystemState> ss_ptr, const bool simulation, const bool development)
  : state_(STATE::OFF)
  , prev_state_(STATE::OFF)
  , operation_mode_(MODE::PRINT)
  , is_sim_(simulation)
  , is_dev_(development)
{
  ss_ = std::move(ss_ptr);
  state_topic_ = "/smalldrop/bioprint/state";
  remote_ctrl_topic_ = "/spacenav/joy";

  setupPublishers();
  publishState();
}

/**
 * \copybrief Bioprinter::publishState()
 */
void Bioprinter::publishState()
{
  std::string state_str;
  switch (state_)
  {
    case STATE::INIT:
      state_str = "INIT";
      break;
    case STATE::IDLE:
      state_str = "IDLE";
      break;
    case STATE::PRINT:
      state_str = "PRINT";
      break;
    case STATE::MOVE:
      state_str = "MOVE";
      break;
    case STATE::PAUSE:
      state_str = "PAUSE";
      break;
    case STATE::ABORT:
      state_str = "ABORT";
      break;
    case STATE::REFILL:
      state_str = "REFILL";
      break;
    case STATE::CALIB_CAM:
      state_str = "CALIB_CAM";
      break;
    case STATE::CALIB_PHEAD:
      state_str = "CALIB_PHEAD";
      break;
    case STATE::ERROR:
      state_str = "ERROR";
      break;
    default:
      state_str = "OFF";
      break;
  }

  std_msgs::String msg;
  msg.data = state_str;
  state_pub_.publish(msg);
}

/**
 * \copybrief Bioprinter::getCurrentState()
 */
STATE Bioprinter::getCurrentState() const
{
  return state_;
}

/**
 * \copybrief Bioprinter::getPrevState()
 */
STATE Bioprinter::getPrevState() const
{
  return prev_state_;
}

/**
 * \copybrief Bioprinter::setState(STATE new_state)
 */
void Bioprinter::setState(STATE new_state)
{
  prev_state_ = state_;
  state_ = new_state;
}

/**
 * \copybrief Bioprinter::isSimulation() const
 */
bool Bioprinter::isSimulation() const
{
  return is_sim_;
}

/**
 * \copybrief Bioprinter::isDevelopment()
 */
bool Bioprinter::isDevelopment() const
{
  return is_dev_;
}

/**
 * \copybrief Bioprinter::init()
 */
void Bioprinter::init(const bool calib_cam, const bool calib_phead, const bool calib_only)
{
  if (!calib_only)
  {
    // Read system configurations

    if (!initRobotArm())
      return;  // initRobotArm sets STATE::ERROR
  }
  // init print head
  // init camera
  if (!calib_only)
  {
    if (!initRemoteController())
      return; // initRemoteController sets STATE::ERROR
  }

  // If everything went well, change state to IDLE
  setState(STATE::IDLE);
}

/**
 * \copybrief Bioprinter::shutdown()
 */
void Bioprinter::shutdown()
{
  // Run remote controller shutdown sequence
  shutdownRemoteController();

  // Run camera shutdown sequence

  // Run print head shutdown sequence

  // Run robot arm shutdown sequence
  shutdownRobotArm();

  // Set the state to OFF
  setState(STATE::OFF);
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief Bioprinter::setupPublishers()
 */
void Bioprinter::setupPublishers()
{
  state_pub_ = nh_.advertise<std_msgs::String>(state_topic_, 10);
}

/**
 * \copybrief Bioprinter::initRobotArm()
 */
bool Bioprinter::initRobotArm()
{
  std::stringstream launch_cmd;
  launch_cmd << "roslaunch smalldrop_robot_arm";

  if (is_sim_)
    launch_cmd << " cartesian_impedance_sim_controller.launch";
  else
    launch_cmd << " cartesian_impedance_real_controller.launch robot_ip:=172.16.0.2";

  if (is_dev_)
    launch_cmd << " rviz:=1 gains:=1";

  // launch_cmd << " &";  // To launch and detach from thread, otherwise caller will be blocked.

  int ret = std::system(launch_cmd.str().c_str());

  if (ret != 0)
  {
    setState(STATE::ERROR);
    return false;
  }

  return true;
}

/**
 * \copybrief Bioprinter::shutdownRobotArm()
 */
void Bioprinter::shutdownRobotArm()
{
  // Change to joint controller

  // Move to shutdown position
}

/**
 * \copybrief Bioprinter::initRemoteController()
 */
bool Bioprinter::initRemoteController()
{
  action_map_t action_map_default, action_map_teleop, action_map_comanip;
  button_map_t button_map_default, button_map_teleop, button_map_comanip;

  action_map_default = { 
    { "mode", boost::bind(&teleop_actions::changeMode, _1) },
  };
  button_map_default = {
    { "mode", "buttons_0" },
  };
  action_map_teleop = { 
    { "mode", boost::bind(&teleop_actions::changeMode, _1) },
    { "joy", boost::bind(&teleop_actions::moveRobotArm, _1) },
  };
  button_map_teleop = {
    { "mode", "buttons_0" },
    { "joy", "axes_*" },
  };
  action_map_comanip = { 
    { "mode", boost::bind(&teleop_actions::changeMode, _1) },
    { "joy", boost::bind(&teleop_actions::moveRobotArm, _1) },
    { "ok", boost::bind(&teleop_actions::publishSegmentationPoint, _1) } 
  };
  button_map_comanip = {
    { "joy", "axes_*" },
    { "mode", "buttons_0" },
    { "ok", "buttons_1" },
  };

  // Setup modes
  IRemoteControllerMode* mode_default = new RemoteControllerMode("default", button_map_default, action_map_default);
  IRemoteControllerMode* mode_teleop = new RemoteControllerMode("teleop", button_map_teleop, action_map_teleop);
  IRemoteControllerMode* mode_comanip = new RemoteControllerMode("comanip", button_map_comanip, action_map_comanip);
  std::list<IRemoteControllerMode*> modes = { mode_default, mode_teleop, mode_comanip };

  std::unique_ptr<IRemoteController> sm(new SpaceMouse(remote_ctrl_topic_, modes, ss_.get()));
  remote_ctrl_ptr_ = std::move(sm);

  if (!remote_ctrl_ptr_->turnOn())
  {
    setState(STATE::ERROR);
    return false;
  }

  return true;
}

/**
 * \copybrief Bioprinter::shutdownRemoteController()
 */
void Bioprinter::shutdownRemoteController()
{
  remote_ctrl_ptr_->turnOff();
}

}  // namespace smalldrop_bioprint

}  // namespace smalldrop