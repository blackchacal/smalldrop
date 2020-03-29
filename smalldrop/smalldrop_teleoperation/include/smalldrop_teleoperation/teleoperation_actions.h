// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file teleoperation_actions.h
 * \brief Declares a set of functions (actions) for robot arm teleoperation.
 */

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

bool moveRobotArm(smalldrop_bioprint::SystemState* system_state);

}  // namespace teleop_actions

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop