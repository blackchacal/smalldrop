// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file spacemouse.h
 * \brief Declares class for the space mouse remote controller.
 */

#ifndef _SMALLDROP_SPACEMOUSE_REMOTE_CONTROLLER_H
#define _SMALLDROP_SPACEMOUSE_REMOTE_CONTROLLER_H

#include <ros/ros.h>
#include <ros/master.h>

// ROS messages
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Pose.h>

#include <smalldrop_teleoperation/i_remote_controller_mode.h>
#include <smalldrop_teleoperation/remote_controller.h>

namespace smalldrop
{
namespace smalldrop_teleoperation
{
class SpaceMouse : public RemoteController
{
private:
  /**
   * Class members
   *****************************************************************************************/

  // State variables
  sensor_msgs::Joy state_;          /** \var Stores the state of space mouse controller gathered from a ROS topic. */
  sensor_msgs::Joy state_prev_;     /** \var Stores the previous state of space mouse controller gathered from a ROS topic. */

  smalldrop_state::SystemState* system_state_;

  // ROS Topics
  std::string state_topic_;       /** \var ROS topic where space mouse state is published. */

  ros::NodeHandle nh_; /** \var ROS node handle to access topics system. */
  ros::Publisher pub_; /** \var ROS publisher to access topics system. */

  // Publishers
  ros::Subscriber state_sub_;       /** \var ROS topic subscriber instance to subscribe to space mouse state topic. */

  bool continuous_action_; /** \var Flag to activate continuous action call. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \copydoc RemoteController::connect()
   */
  virtual void connect() override;

  /**
   * \copydoc RemoteController::disconnect()
   */
  virtual void disconnect() override;

  /**
   * \fn void subscribeToStateTopic()
   * \brief Handles state topic subscription.
   */
  void subscribeToStateTopic();

  /**
   * \fn void getStateFromTopic(const sensor_msgs::Joy::ConstPtr &msg)
   * \brief Reads the controller state from the defined topic.
   *
   * \param msg Topic message with the controller state data.
   */
  void getStateFromTopic(const sensor_msgs::Joy::ConstPtr &msg);

  /**
   * \fn void checkButtonPress()
   * \brief Checks if any of the configure buttons changed state and call the associated action.
   */
  void checkButtonPress();

public:
  /**
   * \fn SpaceMouse(std::string topic, smalldrop_state::SystemState* system_state)
   * \brief Constructor that only sets the state topic. Leaves all modes empty.
   *
   * \param topic Controller state data topic.
   * \param system_state SystemState class pointer. It provides access to all the system data.
   */
  SpaceMouse(std::string topic, smalldrop_state::SystemState* system_state);

  /**
   * \fn SpaceMouse(std::string topic, std::list<IRemoteControllerMode *> modes, smalldrop_state::SystemState* system_state)
   * \brief Constructor that sets the state topic and also the modes.
   *
   * \param topic Controller state data topic.
   * \param modes List of key map modes.
   * \param system_state SystemState class pointer. It provides access to all the system data.
   */
  SpaceMouse(std::string topic, std::list<IRemoteControllerMode *> modes, smalldrop_state::SystemState* system_state);

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn std::string getStateTopic() const
   * \brief Returns the state topic.
   */
  std::string getStateTopic() const;

  /**
   * \fn sensor_msgs::Joy& getState()
   * \brief Returns the current controller state.
   */
  sensor_msgs::Joy& getState();
};

}  // namespace smalldrop_teleoperation

}  // namespace smalldrop

#endif  // _SMALLDROP_SPACEMOUSE_REMOTE_CONTROLLER_H