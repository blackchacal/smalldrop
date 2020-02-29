/**
 * \file spatial_2d_processor.h
 * \brief Header file for Spatial2DProcessor class. 
 */

#ifndef _SPATIAL_2D_PROCESSOR_H
#define _SPATIAL_2D_PROCESSOR_H

#include <ros/ros.h>
#include "opencv2/imgproc.hpp"
#include <iostream>

/**
 * \namespace vision_system
 * \brief Namespace for all the classes related to the project vision system
 */
namespace vision_system
{

/**
 * \class Spatial2DProcessor
 * \brief The class is responsible calculating spatial information on 2D wound contours. 
 */
class Spatial2DProcessor
{
  private:

    // Vars
    unsigned int imageWidth = 500; /** \var Image width in px. */ 
    unsigned int imageHeight = 500; /** \var Image height in px. */
    double xCoordMinLimit = 0.0; /** \var robot coordinates minimum x limit. */
    double xCoordMaxLimit = 1.0; /** \var robot coordinates maximum x limit. */
    double yCoordMinLimit = -0.5; /** \var robot coordinates minimum y limit. */
    double yCoordMaxLimit = 0.5; /** \var robot coordinates maximum y limit. */

  public:
    /**
     * \brief Default class constructor.
     */
    Spatial2DProcessor();

    /**
     * \brief Class constructor where robot and image limits are defined.
     * \param imWidth Image width in pixels
     * \param imHeight Image height in pixels
     * \param xmin Robot coordinates minimum x limit
     * \param xmax Robot coordinates maximum x limit
     * \param ymin Robot coordinates minimum y limit
     * \param ymax Robot coordinates maximum y limit
     */
    Spatial2DProcessor(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax);

    /**
     * \fn double convPx2Meter(void)
     * \brief Convert the pixel size to meters
     */
    std::vector<double> convPx2Meter(void);

    /**
     * \fn double convPxSq2MeterSq(void)
     * \brief Convert the pixel squared area to meter squared.
     */
    double convPxSq2MeterSq(void);

    /**
     * \fn double calcWoundContourArea(std::vector<cv::Point> contour)
     * \brief Calculates the wound contour area in real world dimensions.
     * \param contour List of opencv points that form a wound contour.
     */
    double calcWoundContourArea(std::vector<cv::Point> contour);

    /**
     * \fn double calcWoundContourPerimeter(std::vector<cv::Point> contour)
     * \brief Calculates the wound contour perimeter in real world dimensions. 
     *        It assumes the pixel is square.
     * \param contour List of opencv points that form a wound contour.
     */
    double calcWoundContourPerimeter(std::vector<cv::Point> contour);
};

}

#endif // _SPATIAL_2D_PROCESSOR_H