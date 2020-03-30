// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file teleoperation_actions.h
 * \brief Declares a set of functions (actions) for robot arm teleoperation.
 */

#include <fstream>

#include <smalldrop_bioprint/system_state.h>

// Libraries
#include <cmath>
#include <Eigen/Dense>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
namespace teleop_actions
{

/**
 * \fn bool changeMode(smalldrop_bioprint::SystemState* system_state)
 * \brief Executes actions when changing the remote controller mode.
 * 
 * \param system_state SystemState class pointer. It provides access to all the system data.
 * 
 * This function should be associated with the button configured to change the remote 
 * controller modes. It exists to execute cleaning code associated with mode change.
 * Examples:
 * 1. Clean the wound segmentation points file
 */
bool changeMode(smalldrop_bioprint::SystemState* system_state);

/**
 * \fn bool moveRobotArm(smalldrop_bioprint::SystemState* system_state)
 * \brief Publish robot arm desired posed when moving the remote controller.
 * 
 * \param system_state SystemState class pointer. It provides access to all the system data.
 */
bool moveRobotArm(smalldrop_bioprint::SystemState* system_state);

/**
 * \fn bool publishSegmentationPoint(smalldrop_bioprint::SystemState* system_state)
 * \brief Publish the wound segmentation points for co-manipulation mode.
 * 
 * \param system_state SystemState class pointer. It provides access to all the system data.
 */
bool publishSegmentationPoint(smalldrop_bioprint::SystemState* system_state);

}  // namespace teleop_actions

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop