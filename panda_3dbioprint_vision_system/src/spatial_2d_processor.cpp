#include <panda_3dbioprint_vision_system/spatial_2d_processor.h>

namespace vision_system
{

/*----------------------------------------------------------------------------------------
 *----------------------------------------------------------------------------------------
 * 
 * Private methods
 * 
 *----------------------------------------------------------------------------------------
 *---------------------------------------------------------------------------------------*/

/**
 * \fn double convPx2Meter(void)
 * \brief Convert the pixel size to meters
 */
std::vector<double> Spatial2DProcessor::convPx2Meter(void)
{
  double pxW = (yCoordMaxLimit - yCoordMinLimit) / imageWidth;
  double pxH = (xCoordMaxLimit - xCoordMinLimit) / imageHeight;
  std::vector<double> pxDim;
  pxDim.push_back(pxW);
  pxDim.push_back(pxH);
  return pxDim;
}

/**
 * \fn double convPxSq2MeterSq(void)
 * \brief Convert the pixel squared area to meter squared.
 */
double Spatial2DProcessor::convPxSq2MeterSq(void)
{
  std::vector<double> pxDim = convPx2Meter();
  return pxDim[0]*pxDim[1];
}



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
Spatial2DProcessor::Spatial2DProcessor() {}

/**
 * \brief Class constructor where robot and image limits are defined.
 * \param imWidth Image width in pixels
 * \param imHeight Image height in pixels
 * \param xmin Robot coordinates minimum x limit
 * \param xmax Robot coordinates maximum x limit
 * \param ymin Robot coordinates minimum y limit
 * \param ymax Robot coordinates maximum y limit
 *---------------------------------------------------------------------------------------*/
Spatial2DProcessor::Spatial2DProcessor(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax)
{
  imageWidth = imWidth;
  imageHeight = imHeight;
  xCoordMinLimit = xmin;
  xCoordMaxLimit = xmax;
  yCoordMinLimit = ymin;
  yCoordMaxLimit = ymax; 
}

/**
 * \fn double calcWoundContourArea(std::vector<cv::Point> contour)
 * \brief Calculates the wound contour area in real world dimensions.
 * \param contour List of opencv points that form a wound contour.
 *---------------------------------------------------------------------------------------*/
double Spatial2DProcessor::calcWoundContourArea(std::vector<cv::Point> contour)
{
  if (contour.size() > 3)
    return cv::contourArea(contour) * convPxSq2MeterSq();
  else
    return 0;
}

/**
 * \fn double calcWoundContourPerimeter(std::vector<cv::Point> contour)
 * \brief Calculates the wound contour perimeter in real world dimensions.
 *        It assumes the pixel is square.
 * \param contour List of opencv points that form a wound contour.
 *---------------------------------------------------------------------------------------*/
double Spatial2DProcessor::calcWoundContourPerimeter(std::vector<cv::Point> contour)
{
  std::vector<double> pxDim = convPx2Meter();
  if (contour.size() > 1)
    return cv::arcLength(contour, true) * pxDim[0];
  else
    return 0;
}

}