// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file i_remote_controller.h
 * \brief Declares interface for remote controllers.
 */

#ifndef _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_H
#define _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_H

#include <list>
#include <smalldrop_teleoperation/i_remote_controller_mode.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{

/**
 * \class IRemoteController
 * \brief Interface for remote controllers for teleoperation.
 */
class IRemoteController
{
  public:
    virtual ~IRemoteController() {}

    /**
     * Class methods
     *****************************************************************************************/

    /**
     * \fn virtual bool turnOn()
     * \brief Turns the remote controller on, ready to operate.
     */
    virtual bool turnOn() = 0;

    /**
     * \fn virtual bool turnOff()
     * \brief Turns the remote controller off, end all operations.
     */
    virtual bool turnOff() = 0;

    /**
     * \fn virtual bool isConnected()
     * \brief Checks if the remote controller is connected to the system.
     */
    virtual bool isConnected() const = 0;

    /**
     * \fn virtual bool configModes(std::list<IRemoteControllerMode> modes)
     * \brief Configures the controller modes, i.e., key-action maps.
     * 
     * \param modes List of modes to configure.
     */
    virtual bool configModes(std::list<IRemoteControllerMode*> modes) = 0;
};

} // namespace smalldrop_teleoperation

} // namespace smalldrop

#endif // _SMALLDROP_INTERFACE_REMOTE_CONTROLLER_H