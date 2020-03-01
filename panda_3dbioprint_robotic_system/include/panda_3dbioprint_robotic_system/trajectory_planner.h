/**
 * \file trajectory_planner.h
 * \brief Header file for TrajectoryPlanner class.
 */

#ifndef _TRAJECTORY_PLANNER_H
#define _TRAJECTORY_PLANNER_H

#include <panda_3dbioprint_robotic_system/toolpath_planner.h>
#include <ros/ros.h>
#include <trajectory_planner/cartesian_space_trajectory_planner.h>

/**
 * \namespace robotic_system
 * \brief Namespace for all the classes related to the project robotic system
 */
namespace robotic_system
{
/**
 * \class TrajectoryPlanner
 * \brief The class is responsible planning the trajectory for wound filling.
 */
class TrajectoryPlanner
{
private:
  // Vars
  unsigned int imageWidth = 500;            /** \var Image width in px. */
  unsigned int imageHeight = 500;           /** \var Image height in px. */
  double xCoordMinLimit = 0.0;              /** \var robot coordinates minimum x limit. */
  double xCoordMaxLimit = 1.0;              /** \var robot coordinates maximum x limit. */
  double yCoordMinLimit = -0.5;             /** \var robot coordinates minimum y limit. */
  double yCoordMaxLimit = 0.5;              /** \var robot coordinates maximum y limit. */
  vision_system::WoundSegmentation wseg;    /** \var Wound Segmentation object instance */
  ToolpathPlanner pl;                       /** \var Toolpath planner object instance */

public:
  /**
   * \brief Default class constructor.
   */
  TrajectoryPlanner();

  /**
   * \brief Class constructor where robot and image limits are defined.
   * \param imWidth Image width in pixels
   * \param imHeight Image height in pixels
   * \param xmin Robot coordinates minimum x limit
   * \param xmax Robot coordinates maximum x limit
   * \param ymin Robot coordinates minimum y limit
   * \param ymax Robot coordinates maximum y limit
   */
  TrajectoryPlanner(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax);

  /**
   * \fn std::vector<geometry_msgs::Pose> planTrajectory(const std::string filepath, const double duration, 
   *                                               const double frequency = 100, const PATH_TYPE path_type = PATH_TYPE::PARALLEL_LINES,
   *                                               const unsigned int offset_x = 5, const unsigned int offset_y = 5, const double pose_z = 0.05, 
   *                                               const IMAGE_AXIS axis = IMAGE_AXIS::X,
   *                                               const trajectory_planner::PLAN_MODE mode = trajectory_planner::PLAN_MODE::POLY3);
   * \brief Plans a robot trajectory for wound filling given the path type, duration, frequency and planning mode.
   * \param filepath The path to the file with wound segmentation poses data.
   * \param duration Trajectory execution duration.
   * \param frequency Frequency at which the poses will be publish to the robot.
   * \param path_type Path type name.
   * \param offset_x Distance between grid lines parallel to x axis.
   * \param offset_y Distance between grid lines parallel to y axis.
   * \param pose_z Z axis coordinate for the robot path execution.
   * \param axis The axis the is parallel to the grid lines.
   * \param mode Trajectory planning mode (POLY3 or LSPB)
   */
  std::vector<geometry_msgs::Pose> planTrajectory(const std::string filepath, const double duration, 
                                                  const double frequency = 100, const PATH_TYPE path_type = PATH_TYPE::PARALLEL_LINES,
                                                  const unsigned int offset_x = 5, const unsigned int offset_y = 5, const double pose_z = 0.05, 
                                                  const IMAGE_AXIS axis = IMAGE_AXIS::X,
                                                  const trajectory_planner::PLAN_MODE mode = trajectory_planner::PLAN_MODE::POLY3);
};

}  // namespace robotic_system

#endif  // _TRAJECTORY_PLANNER_H