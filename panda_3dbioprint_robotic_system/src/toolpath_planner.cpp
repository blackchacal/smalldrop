#include <panda_3dbioprint_robotic_system/toolpath_planner.h>

namespace robotic_system
{

/*----------------------------------------------------------------------------------------
 *----------------------------------------------------------------------------------------
 * 
 * Private methods
 * 
 *----------------------------------------------------------------------------------------
 *---------------------------------------------------------------------------------------*/

/**
 * \fn std::vector<std::vector<cv::Point>> getGridLines(cv::Rect bounding_box, IMAGE_AXIS axis = IMAGE_AXIS::X)
 * \brief Obtains the points of the bounding box that form the grid parallel lines.
 * \param bounding_box Bounding box of a contour.
 * \param offset Distance between grid lines.
 * \param axis The axis the is parallel to the grid lines.
 */
std::vector<std::vector<cv::Point>> ToolpathPlanner::getGridLines(cv::Rect bounding_box, unsigned int offset, IMAGE_AXIS axis)
{
  unsigned int increment = 0;
  unsigned int nlines;
  unsigned int padding = 10;
  std::vector<std::vector<cv::Point>> lines;

  if (offset == 0) offset = 5;

  if (axis == IMAGE_AXIS::X)
    nlines = ceil((bounding_box.height + 2*padding) / offset);
  else
    nlines = ceil((bounding_box.width + 2*padding) / offset);

  for (size_t i = 0; i <= nlines; i++)
  {
    std::vector<cv::Point> line;
    if (axis == IMAGE_AXIS::X)
    {
      cv::Point pt1(bounding_box.x - padding, bounding_box.y - padding + increment);
      line.push_back(pt1);
      cv::Point pt2(bounding_box.x + bounding_box.width + padding, bounding_box.y - padding + increment);
      line.push_back(pt2);
    }
    else
    {
      cv::Point pt1(bounding_box.x - padding + increment, bounding_box.y - padding);
      line.push_back(pt1);
      cv::Point pt2(bounding_box.x - padding + increment, bounding_box.y + bounding_box.height + padding);
      line.push_back(pt2);
    }
    lines.push_back(line);
    increment += offset;
  }
  return lines;
}

/**
 * \fn void getComplexContours(const std::vector<cv::Point> contour, std::vector<std::vector<cv::Point>> &complex_contours, cv::Mat &img)
 * \brief Returns a more complex contour (more points) for proper grid lines intersection calculation.
 * \param contour List of opencv points that form a wound contour.
 */
void ToolpathPlanner::getComplexContours(const std::vector<cv::Point> contour, 
                                         std::vector<std::vector<cv::Point>> &complex_contours, 
                                         cv::Mat &img)
{
  // Get new contour with more points to find more interceptions with the grid lines
  std::vector<std::vector<cv::Point>> simple_contours;
  simple_contours.push_back(contour);
  cv::drawContours(img, simple_contours, -1, cv::Scalar(100), 1, cv::LINE_AA);
  cv::findContours(img, complex_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
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
ToolpathPlanner::ToolpathPlanner() 
{
  spc = vision_system::Spatial2DProcessor(imageWidth, imageHeight, xCoordMinLimit, xCoordMaxLimit, yCoordMinLimit, yCoordMaxLimit);
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
ToolpathPlanner::ToolpathPlanner(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax)
{
  imageWidth = imWidth;
  imageHeight = imHeight;
  xCoordMinLimit = xmin;
  xCoordMaxLimit = xmax;
  yCoordMinLimit = ymin;
  yCoordMaxLimit = ymax;
  spc = vision_system::Spatial2DProcessor(imWidth, imHeight, xmin, xmax, ymin, ymax);
}

/**
 * \fn std::vector<cv::Point> genToolpathZigZag(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
 * \brief Returns a wound filling zig zag path.
 * \param contour List of opencv points that form a wound contour.
 * \param offset Distance between grid lines.
 * \param axis The axis the is parallel to the grid lines.
 */
std::vector<cv::Point> ToolpathPlanner::genToolpathZigZag(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis)
{
  std::vector<cv::Point> path_points;
  cv::Rect bounding_box = cv::boundingRect(contour);
  std::vector<std::vector<cv::Point>> lines = getGridLines(bounding_box, offset, axis);

  cv::Mat img(imageWidth, imageHeight, CV_8UC1);
  img = cv::Scalar::all(0);

  // Get complex contours to properly intersection calculation with grid lines
  std::vector<std::vector<cv::Point>> complex_contours;
  getComplexContours(contour, complex_contours, img);

  // For every grid line find the interception points with the contour
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::LineIterator it(img, lines[i].front(), lines[i].back(), 4);
    std::vector<cv::Point> line_interceptions;
    for (size_t j = 0; j < it.count; j++, ++it)
    {
      if (cv::pointPolygonTest(complex_contours[0], it.pos(), false) == 0)
        line_interceptions.push_back(it.pos());
    }
    // Only select the two extreme interception points
    if (line_interceptions.size() > 0)
      path_points.push_back(line_interceptions.front());
    if (line_interceptions.size() > 1)
      path_points.push_back(line_interceptions.back());
  }
  return path_points;
}

/**
 * \fn std::vector<cv::Point> genToolpathParallelLines(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis = IMAGE_AXIS::X)
 * \brief Returns a wound filling parallel path.
 * \param contour List of opencv points that form a wound contour.
 * \param offset Distance between grid lines.
 * \param axis The axis the is parallel to the grid lines.
 */
std::vector<cv::Point> ToolpathPlanner::genToolpathParallelLines(std::vector<cv::Point> contour, unsigned int offset, IMAGE_AXIS axis)
{
  std::vector<cv::Point> path_points;
  cv::Rect bounding_box = cv::boundingRect(contour);
  std::vector<std::vector<cv::Point>> lines = getGridLines(bounding_box, offset, axis);

  cv::Mat img(imageWidth, imageHeight, CV_8UC1);
  img = cv::Scalar::all(0);

  // Get complex contours to properly intersection calculation with grid lines
  std::vector<std::vector<cv::Point>> complex_contours;
  getComplexContours(contour, complex_contours, img);

  // For every grid line find the interception points with the contour.
  // Order the points so that it follows a parallel path
  for (size_t i = 0; i < lines.size(); i++)
  {
    cv::LineIterator it(img, lines[i].front(), lines[i].back(), 4);
    std::vector<cv::Point> line_interceptions;
    for (size_t j = 0; j < it.count; j++, ++it)
    {
      if (cv::pointPolygonTest(complex_contours[0], it.pos(), false) == 0)
        line_interceptions.push_back(it.pos());
    }
    // Only select the two extreme interception points
    if (line_interceptions.size() == 1)
      path_points.push_back(line_interceptions.front());
    else if (line_interceptions.size() > 1)
    {
      // If i is odd switch the order of the points
      if (i % 2 == 0)
      {
        path_points.push_back(line_interceptions.front());
        path_points.push_back(line_interceptions.back());
      }
      else
      {
        path_points.push_back(line_interceptions.back());
        path_points.push_back(line_interceptions.front());
      }
    }
  }
  return path_points;
}

/**
 * \fn std::vector<cv::Point> genToolpathGrid(std::vector<cv::Point> contour, unsigned int offset_x, unsigned int offset_y)
 * \brief Returns a wound filling x,y grid path.
 * \param contour List of opencv points that form a wound contour.
 * \param offset_x Distance between grid lines parallel to x axis.
 * \param offset_y Distance between grid lines parallel to y axis.
 */
std::vector<cv::Point> ToolpathPlanner::genToolpathGrid(std::vector<cv::Point> contour, unsigned int offset_x, unsigned int offset_y)
{
  std::vector<cv::Point> final_path;
  std::vector<cv::Point> path_x = genToolpathParallelLines(contour, offset_x, IMAGE_AXIS::X);
  std::vector<cv::Point> path_y = genToolpathParallelLines(contour, offset_y, IMAGE_AXIS::Y);
  std::reverse(std::begin(path_y), std::end(path_y));

  // Create new vector by concatenation
  final_path.reserve(path_x.size() + path_y.size());
  final_path.insert(final_path.end(), path_x.begin(), path_x.end());
  final_path.insert(final_path.end(), path_y.begin(), path_y.end());

  return final_path;
}

/**
 * \fn std::vector<geometry_msgs::Pose> convPathPoint2Pose(std::vector<cv::Point> path, double pose_z)
 * \brief Returns a wound filling as robot poses instead of opencv points.
 * \param path List of opencv points that form a wound filling path.
 * \param pose_z Z axis coordinate for the robot path execution.
 */
std::vector<geometry_msgs::Pose> ToolpathPlanner::convPathPoint2Pose(std::vector<cv::Point> path, double pose_z)
{
  std::vector<geometry_msgs::Pose> new_path;
  std::vector<double> pxDim = spc.convPx2Meter(); // pixel dimensions in m

  for (size_t i = 0; i < path.size(); i++)
  {
    geometry_msgs::Pose pose;
    pose.position.x = xCoordMinLimit + path[i].y * pxDim[1];
    pose.position.y = yCoordMinLimit + path[i].x * pxDim[0];
    pose.position.z = pose_z;
    pose.orientation.x = 1.0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    new_path.push_back(pose);
  }
  return new_path;
}

}