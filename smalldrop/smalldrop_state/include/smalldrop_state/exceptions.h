// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file exceptions.h
 * \brief Declares exceptions classes for the whole smalldrop system.
 */

#ifndef _SMALLDROP_EXCEPTIONS_H
#define _SMALLDROP_EXCEPTIONS_H

#include <exception>
#include <string>

namespace smalldrop
{
namespace smalldrop_state
{
class SmallDropException : public std::exception
{
protected:
  std::string type_ = "SmallDrop";

public:
  virtual const char* what() const throw()
  {
    return "SmallDrop Exception.";
  }

  std::string getType() const
  {
    return type_;
  }
};

class RobotArmInitException : public SmallDropException
{
public:
  RobotArmInitException() 
  {
    type_ = "RobotArmInit";
  }

  virtual const char* what() const throw()
  {
    return "Robot Initalisation Failure Exception.";
  }
};

class RemoteCtrlInitException : public SmallDropException
{
public:
  RemoteCtrlInitException() 
  {
    type_ = "RemoteCtrlInit";
  }

  virtual const char* what() const throw()
  {
    return "Remote Controller Initalisation Failure Exception.";
  }
};

class ConfigFileSaveException : public SmallDropException
{
public:
  ConfigFileSaveException() 
  {
    type_ = "ConfigFileSave";
  }

  virtual const char* what() const throw()
  {
    return "System Configuration File Write Failure Exception.";
  }
};

class TrajectoryMaxSpeedExceededException : public SmallDropException
{
public:
  TrajectoryMaxSpeedExceededException() 
  {
    type_ = "TrajectoryMaxSpeedExceeded";
  }

  virtual const char* what() const throw()
  {
    return "Trajectory Maximum Speed Exceeded Exception.";
  }
};

class JointTrajectoryLSPBInvalidAccelerationException : public SmallDropException
{
public:
  JointTrajectoryLSPBInvalidAccelerationException() 
  {
    type_ = "JointTrajectoryLSPBInvalidAcceleration";
  }

  virtual const char* what() const throw()
  {
    return "Invalid Acceleration on LSPB Joint Trajectory Planning Exception.";
  }
};

class InvalidNumberOfJointsException : public SmallDropException
{
public:
  InvalidNumberOfJointsException() 
  {
    type_ = "InvalidNumberOfJoints";
  }

  virtual const char* what() const throw()
  {
    return "Invalid Number of Joints Exception.";
  }
};

class RobotArmControllerSwitchException : public SmallDropException
{
public:
  RobotArmControllerSwitchException() 
  {
    type_ = "RobotArmControllerSwitch";
  }

  virtual const char* what() const throw()
  {
    return "Robot Arm Controller Switch Exception.";
  }
};

}  // namespace smalldrop_state

}  // namespace smalldrop

#endif  // _SMALLDROP_EXCEPTIONS_H