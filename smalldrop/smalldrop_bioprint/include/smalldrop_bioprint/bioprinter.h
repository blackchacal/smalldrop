// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file bioprinter.h
 * \brief Declares a class that controls the whole smalldrop system.
 */

#ifndef _SMALLDROP_BIOPRINTER_H
#define _SMALLDROP_BIOPRINTER_H

#include <ros/ros.h>

// ROS messages & services
#include <std_msgs/String.h>
#include <controller_manager_msgs/SwitchController.h>

#include <smalldrop_bioprint/system_config.h>
#include <smalldrop_state/exceptions.h>
#include <smalldrop_teleoperation/i_remote_controller.h>
#include <smalldrop_teleoperation/spacemouse.h>
#include <smalldrop_toolpath/joint_trajectory_planner.h>

using namespace smalldrop::smalldrop_state;
using namespace smalldrop::smalldrop_teleoperation;
using namespace smalldrop::smalldrop_toolpath;

namespace smalldrop
{
namespace smalldrop_bioprint
{

/**
 * \typedef names_t
 */
typedef std::vector<std::string> names_t;

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
public:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn Bioprinter(std::unique_ptr<SystemState> ss_ptr, std::unique_ptr<SystemConfig> config_ptr, const bool simulation, const bool development)
   * \brief Constructor
   *
   * \param ss_ptr SystemState class pointer.
   * \param config_ptr SystemConfig class pointer.
   * \param simulation Boolean that defines if system should operation in simulation or with real robot.
   * \param development Boolean that defines if the system is in development mode or production mode.
   */
  Bioprinter(std::unique_ptr<SystemState> ss_ptr, std::unique_ptr<SystemConfig> config_ptr, const bool simulation, const bool development);

  ~Bioprinter()
  {
  }

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
   * \fn void setErrorState(SmallDropException& exception)
   * \brief Sets the system at error state and updates previous state. It receives an exception
   * object related to the error.
   */
  void setErrorState(SmallDropException& exception);

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

  /**
   * \fn void handleErrors()
   * \brief Manages system errors and deals with error recovery.
   */
  void handleErrors();

private:
  /**
   * Class members
   *****************************************************************************************/

  // State variables
  STATE state_;         /** \var System current state that changes according to work of the system. */
  STATE prev_state_;    /** \var System previous state. It is important for the state machine. */
  MODE operation_mode_; /** \var System operation mode. */

  bool is_sim_;                     /** \var Simulation or real mode. */
  bool is_dev_;                     /** \var Development or production mode. */
  std::unique_ptr<SystemState> ss_; /** \var SystemState instance pointer. It subscribes to all important system topics,
                                       and provides general publishers. */
  std::unique_ptr<SystemConfig> config_; /** \var SystemConfig instance pointer. It allows reading/writing whole system
                                            configuration files. */

  // Error/Exception handling
  SmallDropException last_exception_; /** \var Exception object representing the last system exception. */

  // System components
  std::unique_ptr<IRemoteController> remote_ctrl_ptr_; /** \var Remote Controller instance. */

  // ROS topics
  std::string remote_ctrl_topic_; /** \var ROS topic where remote controller state will be published. */
  std::string state_topic_;       /** \var ROS topic where system working state will be published. */

  // Node handle
  ros::NodeHandle nh_; /** \var ROS node handle to access topics system. */

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
   * \fn bool initRobotArm()
   * \brief Initialize robot arm according to system configuration.
   */
  bool initRobotArm();

  /**
   * \fn void shutdownRobotArm()
   * \brief Shut down the robot arm.
   */
  void shutdownRobotArm();

  /**
   * \fn void moveRobotArmHome()
   * \brief Move the robot arm to the HOME position.
   */
  void moveRobotArmHome();

  /**
   * \fn void switchRobotArmControllers(const names_t start_controllers, const names_t stop_controllers) const;
   * \brief Switches the robot arm controllers.
   */
  void switchRobotArmControllers(const names_t start_controllers, const names_t stop_controllers) const;

  /**
   * \fn bool initRemoteController()
   * \brief Initialize the remote controller according to system configuration.
   */
  bool initRemoteController();

  /**
   * \fn void shutdownRemoteController()
   * \brief Shut down the remote controller.
   */
  void shutdownRemoteController();

  /**
   * \fn void planRobotJointMovement(const double duration, const double frequency, const PLAN_MODE plan_mode, const jpos_t joints_i, const jpos_t joints_f)
   * \brief Plan and send joint configuration to robot.
   * 
   * \param duration Joint movement duration.
   * \param frequency Publishing frequency.
   * \param plan_mode Planning mode.
   * \param joints_i Initial joint configuration.
   * \param joints_f Final joint configuration.
   */
  void planRobotJointMovement(const double duration, const double frequency, const PLAN_MODE plan_mode, const jpos_t joints_i, const jpos_t joints_f);
};

}  // namespace smalldrop_bioprint

}  // namespace smalldrop

#endif  // _SMALLDROP_BIOPRINTER_H