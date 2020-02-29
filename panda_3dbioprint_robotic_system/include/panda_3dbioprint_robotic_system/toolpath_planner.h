/**
 * \file toolpath_planner.h
 * \brief Header file for ToolpathPlanner class. 
 */

#ifndef _TOOLPATH_PLANNER_H
#define _TOOLPATH_PLANNER_H

#include <ros/ros.h>
#include "opencv2/imgproc.hpp"
#include <panda_3dbioprint_vision_system/wound_segmentation.h>
#include <panda_3dbioprint_vision_system/spatial_2d_processor.h>
#include <iostream>
#include <cmath>

/**
 * \namespace robotic_system
 * \brief Namespace for all the classes related to the project robotic system
 */
namespace robotic_system
{

/**
 * \enum IMAGE_AXIS
 * \brief Enum of the image axes
 */
enum class IMAGE_AXIS
{
  X,
  Y
};

/**
 * \class ToolpathPlanner
 * \brief The class is responsible calculating spatial information on 2D wound contours. 
 */
class ToolpathPlanner
{
  private:

    // Vars
    unsigned int imageWidth = 500; /** \var Image width in px. */ 
    unsigned int imageHeight = 500; /** \var Image height in px. */
    double xCoordMinLimit = 0.0; /** \var robot coordinates minimum x limit. */
    double xCoordMaxLimit = 1.0; /** \var robot coordinates maximum x limit. */
    double yCoordMinLimit = -0.5; /** \var robot coordinates minimum y limit. */
    double yCoordMaxLimit = 0.5; /** \var robot coordinates maximum y limit. */
    vision_system::Spatial2DProcessor spc; /** \var 2D spatial processor */

    /**
     * \fn std::vector<std::vector<cv::Point>> getGridLines(cv::Rect bounding_box, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
     * \brief Obtains the points of the bounding box that form the grid parallel lines.
     * \param bounding_box Bounding box of a contour.
     * \param offset Distance between grid lines.
     * \param axis The axis the is parallel to the grid lines.
     */
    std::vector<std::vector<cv::Point>> getGridLines(cv::Rect bounding_box, unsigned int offset, 
                                                     IMAGE_AXIS axis = IMAGE_AXIS::X);

    /**
     * \fn void getComplexContours(const std::vector<cv::Point> contour, std::vector<std::vector<cv::Point>> &complex_contours, cv::Mat &img))
     * \brief Returns a more complex contour (more points) for proper grid lines intersection calculation.
     * \param contour List of opencv points that form a wound contour.
     */
    void getComplexContours(const std::vector<cv::Point> contour, std::vector<std::vector<cv::Point>> &complex_contours, 
                            cv::Mat &img);

  public:
    /**
     * \brief Default class constructor.
     */
    ToolpathPlanner();

    /**
     * \brief Class constructor where robot and image limits are defined.
     * \param imWidth Image width in pixels
     * \param imHeight Image height in pixels
     * \param xmin Robot coordinates minimum x limit
     * \param xmax Robot coordinates maximum x limit
     * \param ymin Robot coordinates minimum y limit
     * \param ymax Robot coordinates maximum y limit
     */
    ToolpathPlanner(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax);

    /**
     * \fn std::vector<cv::Point> genToolpathZigZag(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
     * \brief Returns a wound filling zig zag path.
     * \param contour List of opencv points that form a wound contour.
     * \param offset Distance between grid lines.
     * \param axis The axis the is parallel to the grid lines.
     */
    std::vector<cv::Point> genToolpathZigZag(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X);

    /**
     * \fn std::vector<cv::Point> genToolpathParallelLines(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
     * \brief Returns a wound filling parallel path.
     * \param contour List of opencv points that form a wound contour.
     * \param offset Distance between grid lines.
     * \param axis The axis the is parallel to the grid lines.
     */
    std::vector<cv::Point> genToolpathParallelLines(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X);

    /**
     * \fn std::vector<cv::Point> genToolpathGrid(std::vector<cv::Point> contour, unsigned int offset_x, unsigned int offset_y)
     * \brief Returns a wound filling x,y grid path.
     * \param contour List of opencv points that form a wound contour.
     * \param offset_x Distance between grid lines parallel to x axis.
     * \param offset_y Distance between grid lines parallel to y axis.
     */
    std::vector<cv::Point> genToolpathGrid(std::vector<cv::Point> contour, unsigned int offset_x, unsigned int offset_y);

    /**
     * \fn std::vector<geometry_msgs::Pose> convPathPoint2Pose(std::vector<cv::Point> path, double pose_z)
     * \brief Returns a wound filling as robot poses instead of opencv points.
     * \param path List of opencv points that form a wound filling path.
     * \param pose_z Z axis coordinate for the robot path execution.
     */
    std::vector<geometry_msgs::Pose> convPathPoint2Pose(std::vector<cv::Point> path, double pose_z);
};

}

#endif // _TOOLPATH_PLANNER_H