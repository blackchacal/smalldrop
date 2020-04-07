// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file remote_controller.h
 * \brief Declares class for remote controllers that implements the IRemoteController interface.
 */

#ifndef _SMALLDROP_REMOTE_CONTROLLER_H
#define _SMALLDROP_REMOTE_CONTROLLER_H

#include <smalldrop_teleoperation/i_remote_controller.h>
#include <smalldrop_teleoperation/i_remote_controller_mode.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
/**
 * \class RemoteController
 * \brief Abstract class representing a remote controller for teleoperation.
 */
class RemoteController : public IRemoteController
{
private:
  /**
   * Class members
   *****************************************************************************************/

  std::list<IRemoteControllerMode*>::iterator mode_it_; /** Iterator for controller modes. It points to the active mode. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn void setupButtons()
   * \brief Setup the controller keys based on the active mode.
   */
  void setupButtons();

  /**
   * \fn bool isOnOffButtonSet()
   * \brief Checks if the on/off button was configured.
   */
  bool isOnOffButtonSet() const;

  /**
   * \fn bool isModeButtonSet()
   * \brief Checks if the mode button was configured.
   */
  bool isModeButtonSet() const;

  /**
   * \fn bool isOkButtonSet()
   * \brief Checks if the ok button was configured.
   */
  bool isOkButtonSet() const;

  /**
   * \fn bool isCancelButtonSet()
   * \brief Checks if the cancel button was configured.
   */
  bool isCancelButtonSet() const;

  /**
   * \fn bool isAbortButtonSet()
   * \brief Checks if the abort button was configured.
   */
  bool isAbortButtonSet() const;

  /**
   * \fn bool isUpButtonSet()
   * \brief Checks if the up button was configured.
   */
  bool isUpButtonSet() const;

  /**
   * \fn bool isDownButtonSet()
   * \brief Checks if the down button was configured.
   */
  bool isDownButtonSet() const;

  /**
   * \fn bool isLeftButtonSet()
   * \brief Checks if the left button was configured.
   */
  bool isLeftButtonSet() const;

  /**
   * \fn bool isRightButtonSet()
   * \brief Checks if the right button was configured.
   */
  bool isRightButtonSet() const;

  /**
   * \fn bool isJoyButtonSet()
   * \brief Checks if the joy button was configured.
   */
  bool isJoyButtonSet() const;

protected:
  /**
   * Class members
   *****************************************************************************************/

  bool connected_; /** \var Indicates the connection state of the controller. */

  // Modes
  IRemoteControllerMode* active_mode_;      /** \var Remote controller active mode instance. */
  std::list<IRemoteControllerMode*> modes_; /** \var List of configured modes. */

  // Controller buttons
  std::string on_off_button_; /** \var Stores the keyname associated with the on/off function. */
  std::string mode_button_;   /** \var Stores the keyname associated with the change mode function. */
  std::string ok_button_;     /** \var Stores the keyname associated with the ok function. */
  std::string cancel_button_; /** \var Stores the keyname associated with the cancel function. */
  std::string abort_button_;  /** \var Stores the keyname associated with the abort function. */
  std::string up_button_;     /** \var Stores the keyname associated with the up function. */
  std::string down_button_;   /** \var Stores the keyname associated with the down function. */
  std::string left_button_;   /** \var Stores the keyname associated with the left function. */
  std::string right_button_;  /** \var Stores the keyname associated with the right function. */
  std::string joy_button_;    /** \var Stores the keyname associated with the joy function. */

  /**
   * \brief Protected constructor to prevent class instantiation.
   */
  RemoteController();

  /**
   * \brief Protected constructor to prevent class instantiation.
   *
   * \param modes List of modes to configure.
   */
  RemoteController(std::list<IRemoteControllerMode*> modes);

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn virtual void connect()
   * \brief Establishes the connection with the remote controller.
   */
  virtual void connect() = 0;

  /**
   * \fn virtual void disconnect()
   * \brief Disconnects the remote controller.
   */
  virtual void disconnect() = 0;

  /**
   * \fn bool isButtonSet(std::string button) const
   * \brief Check if a button was set.
   *
   * \param button Name of the button to check if it was activated.
   */
  bool isButtonSet(std::string button) const;

  /**
   * \fn bool callButtonAction(std::string function, smalldrop_state::SystemState* system_state)
   * \brief Call the action associated to the button selected.
   *
   * \param function Function for which the action will be called.
   * \param system_state SystemState class pointer. It provides access to all the system data.
   */
  bool callButtonAction(std::string function, smalldrop_state::SystemState* system_state);

public:
  virtual ~RemoteController() override
  {
    delete active_mode_;
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \copydoc IRemoteController::turnOn()
   */
  virtual bool turnOn() override;

  /**
   * \copydoc IRemoteController::turnOff()
   */
  virtual bool turnOff() override;

  /**
   * \copydoc IRemoteController::isConnected()
   */
  virtual bool isConnected() const override;

  /**
   * \copydoc IRemoteController::configModes()
   */
  virtual bool configModes(std::list<IRemoteControllerMode*> modes) override;

  /**
   * \fn IRemoteControllerMode* whatMode()
   * \brief Returns the active mode.
   */
  IRemoteControllerMode* whatMode() const;

  /**
   * \fn bool isMode(IRemoteControllerMode mode)
   * \brief Checks if the active mode is the one provided.
   *
   * \param mode_name Name to check if corresponds to active mode.
   */
  bool isMode(std::string mode_name) const;

  /**
   * \fn void changeMode()
   * \brief Changes the controller mode in a round-robin fashion.
   */
  void changeMode();
};

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop

#endif  // _SMALLDROP_REMOTE_CONTROLLER_H