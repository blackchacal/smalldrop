// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file i_camera.h
 * \brief Declares interface for general cameras.
 */

#ifndef _SMALLDROP_INTERFACE_CAMERA_H
#define _SMALLDROP_INTERFACE_CAMERA_H

namespace smalldrop
{
namespace smalldrop_vision
{
/**
 * \class ICamera
 * \brief Interface for general cameras.
 */
class ICamera
{
public:
  virtual ~ICamera()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn virtual bool turnOn()
   * \brief Turns the camera on, ready to operate.
   */
  virtual bool turnOn() = 0;

  /**
   * \fn virtual bool turnOff()
   * \brief Turns the camera off, end all operations.
   */
  virtual bool turnOff() = 0;

  /**
   * \fn virtual bool isConnected()
   * \brief Checks if the camera is connected to the system.
   */
  virtual bool isConnected() const = 0;

  /**
   * \fn virtual bool calibrate()
   * \brief Calibrates the camera.
   */
  virtual bool calibrate() = 0;
};

}  // namespace smalldrop_vision
}  // namespace smalldrop

#endif  // _SMALLDROP_INTERFACE_CAMERA_H