// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file i_remote_controller_mode.h
 * \brief Defines IRemoteControllerMode interface for remote controller modes.
 */

#include <map>
#include <functional>

namespace smalldrop
{
namespace smalldrop_teleoperation
{

/**
 * \typedef
 * \brief Typedef for callback functions.
 */
typedef std::function<void()> callback_t;

/**
 * \typedef
 * \brief Typedef for map of controller key names and actions.
 */
typedef std::map<std::string, callback_t> keymap_t;

/**
 * \class IRemoteControllerMode
 * \brief Interface for remote controller modes for teleoperation.
 */
class IRemoteControllerMode
{
  public:
    virtual ~IRemoteControllerMode() {}

    /**
     * Class methods
     *****************************************************************************************/

    /**
     * \fn virtual keymap_t getKeyMap(void)
     * \brief Returns the key map used on the mode.
     * 
     * \param keymap Dictionary matching the key name and key actions.
     */
    virtual keymap_t getKeyMap(void) = 0;

    /**
     * \fn virtual std::function<void(void)> getKeyAction(std::string key)
     * \brief Returns the action function associated with a specified key.
     * 
     * \param key Key name for which the action will be returned.
     */
    virtual std::function<void(void)> getKeyAction(std::string key) = 0;

    /**
     * \fn virtual std::string getName(void)
     * \brief Returns the mode name.
     */
    virtual std::string getName(void) = 0;
};

} // namespace smalldrop_teleoperation

} // namespace smalldrop