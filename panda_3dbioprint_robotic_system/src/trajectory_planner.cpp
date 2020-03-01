#include <panda_3dbioprint_robotic_system/trajectory_planner.h>

namespace robotic_system
{

/*----------------------------------------------------------------------------------------
 *----------------------------------------------------------------------------------------
 * 
 * Private methods
 * 
 *----------------------------------------------------------------------------------------
 *---------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------
 *----------------------------------------------------------------------------------------
 * 
 * Public methods
 * 
 *----------------------------------------------------------------------------------------
 *---------------------------------------------------------------------------------------*/

/**
 * \brief Default class constructor.
 *---------------------------------------------------------------------------------------*/
TrajectoryPlanner::TrajectoryPlanner() 
{
  wseg = vision_system::WoundSegmentation(imageWidth, imageHeight, xCoordMinLimit, xCoordMaxLimit, yCoordMinLimit, yCoordMaxLimit);
  pl = ToolpathPlanner(imageWidth, imageHeight, xCoordMinLimit, xCoordMaxLimit, yCoordMinLimit, yCoordMaxLimit);
}

/**
 * \brief Class constructor where robot and image limits are defined.
 * \param imWidth Image width in pixels
 * \param imHeight Image height in pixels
 * \param xmin Robot coordinates minimum x limit
 * \param xmax Robot coordinates maximum x limit
 * \param ymin Robot coordinates minimum y limit
 * \param ymax Robot coordinates maximum y limit
 *---------------------------------------------------------------------------------------*/
TrajectoryPlanner::TrajectoryPlanner(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax)
{
  imageWidth = imWidth;
  imageHeight = imHeight;
  xCoordMinLimit = xmin;
  xCoordMaxLimit = xmax;
  yCoordMinLimit = ymin;
  yCoordMaxLimit = ymax;
  wseg = vision_system::WoundSegmentation(imWidth, imHeight, xmin, xmax, ymin, ymax);
  pl = ToolpathPlanner(imWidth, imHeight, xmin, xmax, ymin, ymax);
}

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
std::vector<geometry_msgs::Pose> TrajectoryPlanner::planTrajectory(const std::string filepath, const double duration, 
                                                  const double frequency, const PATH_TYPE path_type,
                                                  const unsigned int offset_x, const unsigned int offset_y, const double pose_z,
                                                  const IMAGE_AXIS axis, const trajectory_planner::PLAN_MODE mode)
{
  std::vector<geometry_msgs::Pose> trajectory;
  std::vector<geometry_msgs::Pose> path;
  std::vector<cv::Point> path_points;
  std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);

  switch (path_type)
  {
    case PATH_TYPE::ZIG_ZAG:
      if (axis == IMAGE_AXIS::X)
        path_points = pl.genToolpathZigZag(contour, offset_x, axis);
      else
        path_points = pl.genToolpathZigZag(contour, offset_y, axis);
      break;
    case PATH_TYPE::GRID:
      path_points = pl.genToolpathGrid(contour, offset_x, offset_y);
      break;
    default: // PARALLEL_LINES
      if (axis == IMAGE_AXIS::X)
        path_points = pl.genToolpathParallelLines(contour, offset_x, axis);
      else
        path_points = pl.genToolpathParallelLines(contour, offset_y, axis);
      break;
  }
  path = pl.convPathPoint2Pose(path_points, pose_z);

  for (size_t i = 1; i < path.size(); i++)
  {
    std::vector<geometry_msgs::Pose> sub_trajectory = trajectory_planner::TaskTrajectoryPlanner::linear_trajectory(path[i-1], path[i], duration, frequency, mode); 
    trajectory.insert(trajectory.end(), sub_trajectory.begin(), sub_trajectory.end()); // concatenate vectors
  }
  
  return trajectory;
}                                                  

}