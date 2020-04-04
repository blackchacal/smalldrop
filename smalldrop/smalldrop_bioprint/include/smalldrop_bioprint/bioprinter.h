// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file bioprinter.h
 * \brief Declares a class that controls the whole smalldrop system.
 */

#ifndef _SMALLDROP_BIOPRINTER_H
#define _SMALLDROP_BIOPRINTER_H

#include <ros/ros.h>

// ROS messages
#include <std_msgs/String.h>

namespace smalldrop
{
namespace smalldrop_bioprint
{
/**
 * \enum STATE
 * \brief Defines the various possible states of the system
 */
enum class STATE
{
  OFF,
  INIT,
  IDLE,
  PRINT,
  MOVE,
  PAUSE,
  ABORT,
  REFILL,
  CALIB_CAM,
  CALIB_PHEAD,
  ERROR
};

enum class MODE
{
  PRINT,
  TELEOP,
  COMANIP,
  GCODE
};

class Bioprinter
{
private:
  /**
   * Class members
   *****************************************************************************************/

  // State variables
  STATE state_;           /** \var System current state that changes according to work of the system. */
  STATE prev_state_;      /** \var System previous state. It is important for the state machine. */
  MODE operation_mode_;   /** \var System operation mode. */
  bool is_sim_;           /** \var Simulation or real mode. */
  bool is_dev_;           /** \var Development or production mode. */

  // ROS topics
  std::string state_topic_; /** \var ROS topic where system working state will be published. */

  // Node handle
  ros::NodeHandle nh_;    /** \var ROS node handle to access topics system. */

  // ROS Publishers
  ros::Publisher state_pub_; /** \var Publisher for the system working state. */

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn void setupPublishers()
   * \brief Setup ROS publishers used by the class.
   */
  void setupPublishers();

  /**
   * \fn void initRobotArm()
   * \brief Initialize robot arm according to system configuration.
   */
  bool initRobotArm();

  /**
   * \fn void shutdownRobotArm()
   * \brief Shuts down the robot arm.
   */
  void shutdownRobotArm();

public:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn Bioprinter(const bool simulation, const bool dev)
   * \brief Constructor
   *
   * \param simulation Boolean that defines if system should operation in simulation or with real robot.
   * \param development Boolean that defines if the system is in development mode or production mode.
   */
  Bioprinter(const bool simulation, const bool development);
  
  /**
   * \fn void publishState()
   * \brief Publishes the system working state on a topic.
   */
  void publishState();

  /**
   * \fn STATE getCurrentState() const
   * \brief Returns the current system working state.
   */
  STATE getCurrentState() const;

  /**
   * \fn STATE getPrevState() const
   * \brief Returns the previous system working state.
   */
  STATE getPrevState() const;

  /**
   * \fn void setState(STATE new_state)
   * \brief Sets a new system state and updates previous state.
   */
  void setState(STATE new_state);

  /**
   * \fn bool isSimulation() const
   * \brief Check if the system runs in simulation mode or real mode.
   */
  bool isSimulation() const;

  /**
   * \fn bool isDevelopment() const
   * \brief Checks if system is in development mode or production.
   */
  bool isDevelopment() const;

  /**
   * \fn void init(const bool calib_cam, const bool calib_phead, const bool calib_only)
   * \brief Initialises the whole system (robot arm, print head, camera and remote controllers).
   * 
   * \param calib_cam Informs the system that the camera needs to be calibrated.
   * \param calib_phead Informs the system that the print head needs to be calibrated.
   * \param calib_only If true, skip initialisation of robot arm and remote controllers.
   */
  void init(const bool calib_cam, const bool calib_phead, const bool calib_only);

  /**
   * \fn void shutdown()
   * \brief Shuts down the robot arm, remote controller, camera and print head. It leaves the system
   * on the OFF state.
   */
  void shutdown();

};

}  // namespace smalldrop_bioprint

}  // namespace smalldrop

#endif  // _SMALLDROP_BIOPRINTER_H