// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file remote_controller_mode.h
 * \brief Defines class that represents a remote controller mode.
 */

#include <map>
#include <functional>

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

    std::string mode_name_; /** \var Mode name. */
    keymap_t key_map_; /** Map between remote controller button (key names) and actions (functions). */

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
     * \param keymap Dictionary matching the key name and key actions.
     */
    RemoteControllerMode(std::string name, keymap_t keymap);

    /**
     * \copydoc IRemoteControllerMode::getKeyMap()
     */
    virtual keymap_t getKeyMap(void) override;

    /**
     * \copydoc IRemoteControllerMode::getKeyAction()
     */
    virtual std::function<void(void)> getKeyAction(std::string key) override;

    /**
     * \copydoc IRemoteControllerMode::getName()
     */
    virtual std::string getName(void) override;

    /**
     * \fn void setKeyMap(keymap_t keymap)
     * \brief Sets the key map used on the mode.
     * 
     * \param keymap Dictionary matching the key name and key actions.
     */
    void setKeyMap(keymap_t keymap);
};

} // namespace smalldrop_teleoperation

} // namespace smalldrop