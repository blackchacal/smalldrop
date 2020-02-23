/**
 * \file wound_segmentation.h
 * \brief Header file for WoundSegmentation class. 
 */

#ifndef _WOUND_SEGMENTATION
#define _WOUND_SEGMENTATION

#include <ros/ros.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <fstream>

#include <geometry_msgs/Pose.h>

/**
 * \namespace vision_system
 * \brief Namespace for all the classes related to the project vision system
 */
namespace vision_system
{

/**
 * \class WoundSegmentation
 * \brief The class is responsible for dealing with wound segmentation. 
 *        It deals both with camera and co-manipulation segmentation.
 */
class WoundSegmentation
{
  private:

    // Vars
    unsigned int imageWidth = 500; /** \var Image width in px. */ 
    unsigned int imageHeight = 500; /** \var Image height in px. */
    double xCoordMinLimit = 0.0; /** \var robot coordinates minimum x limit. */
    double xCoordMaxLimit = 1.0; /** \var robot coordinates maximum x limit. */
    double yCoordMinLimit = -0.5; /** \var robot coordinates minimum y limit. */
    double yCoordMaxLimit = 0.5; /** \var robot coordinates maximum y limit. */
    std::fstream fh; /** \var file handler. *

    /**
     * \fn std::vector<geometry_msgs::Pose> loadWoundSegmentationPoints(std::string filepath)
     * \brief Load wound segmentation points from a file.
     * \param filepath The path to the file with wound segmentation poses data.
     */
    std::vector<geometry_msgs::Pose> loadWoundSegmentationPoints(std::string filepath);

    /**
     * \fn cv::Point convPoseToPoint(geometry_msgs::Pose pose)
     * \brief Convert the pose coordinates to the image pixel coordinates.
     * \param pose A geometry_msgs::Pose that is going to be converted to cv::Point.
     */
    cv::Point convPoseToPoint(geometry_msgs::Pose pose);

    /**
     * \fn std::vector<cv::Point> getPointsList(std::string filepath)
     * \brief Creates list of opencv points from wound segmentation points on file
     * \param filepath The path to the file with wound segmentation poses data.
     */
    std::vector<cv::Point> getPointsList(std::string filepath);

  public:
    /**
     * \brief Default class constructor.
     */
    WoundSegmentation();

    /**
     * \brief Class constructor where robot and image limits are defined.
     * \param imWidth Image width in pixels
     * \param imHeight Image height in pixels
     * \param xmin Robot coordinates minimum x limit
     * \param xmax Robot coordinates maximum x limit
     * \param ymin Robot coordinates minimum y limit
     * \param ymax Robot coordinates maximum y limit
     */
    WoundSegmentation(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax);

    /**
     * \fn std::vector<geometry_msgs::Pose> getWoundConvexHullPoses(std::string filepath)
     * \brief Select all the wound segmentation poses that form a convex hull.
     * \param filepath The path to the file with wound segmentation poses data.
     */
    std::vector<geometry_msgs::Pose> getWoundConvexHullPoses(std::string filepath);

    /**
     * \fn std::vector<cv::Point> getWoundConvexHullPoints(std::string filepath)
     * \brief Select all the wound segmentation opencv points that form a convex hull.
     * \param filepath The path to the file with wound segmentation poses data.
     */
    std::vector<cv::Point> getWoundConvexHullPoints(std::string filepath);

    /**
     * \fn void plotWoundConvexHull(std::string filepath)
     * \brief Plot the wound segmentation convex hull.
     * \param filepath The path to the file with wound segmentation poses data.
     */
    void plotWoundConvexHull(std::string filepath);
};

}

#endif // _WOUND_SEGMENTATION