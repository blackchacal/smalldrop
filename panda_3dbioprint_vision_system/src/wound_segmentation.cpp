#include <panda_3dbioprint_vision_system/wound_segmentation.h>

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
 * \fn std::vector<geometry_msgs::Pose> loadWoundSegmentationPoints(std::string filepath)
 * \brief Load wound segmentation points from a file.
 * \param filepath The path to the file with wound segmentation poses data.
 *---------------------------------------------------------------------------------------*/
std::vector<geometry_msgs::Pose> WoundSegmentation::loadWoundSegmentationPoints(std::string filepath)
{
  std::string header;
  std::string x, y, z, ox, oy, oz, ow;
  std::vector<geometry_msgs::Pose> poses;

  fh.open(filepath, std::fstream::in);
  if (fh.is_open())
  {
    getline(fh, header);
    while (getline(fh, x, ' '))
    {
      getline(fh, y, ' ');
      getline(fh, z, ' ');
      getline(fh, ox, ' ');
      getline(fh, oy, ' ');
      getline(fh, oz, ' ');
      getline(fh, ow);
      geometry_msgs::Pose pose;
      pose.position.x = std::stod(x);
      pose.position.y = std::stod(y);
      pose.position.z = std::stod(z);
      pose.orientation.x = std::stod(ox);
      pose.orientation.y = std::stod(oy);
      pose.orientation.z = std::stod(oz);
      pose.orientation.w = std::stod(ow);
      poses.push_back(pose);
    }
    fh.close();
  }
  else
  {
    ROS_ERROR("The file was not properly opened!");
  }
  return poses;
}

/**
 * \fn cv::Point convPoseToPoint(geometry_msgs::Pose pose)
 * \brief Convert the pose coordinates to the image pixel coordinates.
 * \param pose A geometry_msgs::Pose that is going to be converted to cv::Point.
 *---------------------------------------------------------------------------------------*/
cv::Point WoundSegmentation::convPoseToPoint(geometry_msgs::Pose pose)
{
  cv::Point pt;
  pt.x = round( (imageWidth / (yCoordMaxLimit - yCoordMinLimit)) * (pose.position.y - yCoordMinLimit) );
  pt.y = round( (imageHeight / (xCoordMaxLimit - xCoordMinLimit)) * (pose.position.x - xCoordMinLimit) );
  return pt;
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
WoundSegmentation::WoundSegmentation() {}

/**
 * \brief Class constructor where robot and image limits are defined.
 * \param imWidth Image width in pixels
 * \param imHeight Image height in pixels
 * \param xmin Robot coordinates minimum x limit
 * \param xmax Robot coordinates maximum x limit
 * \param ymin Robot coordinates minimum y limit
 * \param ymax Robot coordinates maximum y limit
 *---------------------------------------------------------------------------------------*/
WoundSegmentation::WoundSegmentation(unsigned int imWidth, unsigned int imHeight, double xmin, double xmax, double ymin, double ymax)
{
  imageWidth = imWidth;
  imageHeight = imHeight;
  xCoordMinLimit = xmin;
  xCoordMaxLimit = xmax;
  yCoordMinLimit = ymin;
  yCoordMaxLimit = ymax; 
}

/**
 * \fn std::vector<cv::Point> getPointsList(std::string filepath)
 * \brief Creates list of opencv points from wound segmentation points on file
 * \param filepath The path to the file with wound segmentation poses data.
 *---------------------------------------------------------------------------------------*/
std::vector<cv::Point> WoundSegmentation::getPointsList(std::string filepath)
{
  std::vector<geometry_msgs::Pose> poses = loadWoundSegmentationPoints(filepath);
  std::vector<cv::Point> points;

  for(size_t i = 0; i < poses.size(); i++)
  {
    cv::Point pt = convPoseToPoint(poses[i]);
    points.push_back(pt);
  }
  return points;
}

/**
 * \fn std::vector<geometry_msgs::Pose> getWoundConvexHullPoses(std::string filepath)
 * \brief Select all the wound segmentation poses that form a convex hull.
 * \param filepath The path to the file with wound segmentation poses data.
 *---------------------------------------------------------------------------------------*/
std::vector<geometry_msgs::Pose> WoundSegmentation::getWoundConvexHullPoses(std::string filepath)
{
  std::vector<geometry_msgs::Pose> hull_poses;
  std::vector<geometry_msgs::Pose> poses = loadWoundSegmentationPoints(filepath);
  std::vector<cv::Point> points = getPointsList(filepath);
  
  if (points.size() > 0) 
  {
    std::vector<int> hull;
    cv::convexHull(cv::Mat(points), hull, true);

    for (size_t i = 0; i < hull.size(); i++)
    {
      hull_poses.push_back(poses[hull[i]]);
    }
  }

  return hull_poses;
}

/**
 * \fn std::vector<cv::Point> getWoundConvexHullPoints(std::string filepath)
 * \brief Select all the wound segmentation opencv points that form a convex hull.
 * \param filepath The path to the file with wound segmentation poses data.
 *---------------------------------------------------------------------------------------*/
std::vector<cv::Point> WoundSegmentation::getWoundConvexHullPoints(std::string filepath)
{
  std::vector<cv::Point> points = getPointsList(filepath);
  std::vector<cv::Point> hull;

  if (points.size() > 0) 
    cv::convexHull(cv::Mat(points), hull, true);

  return hull;
}

/**
 * \fn void plotWoundConvexHull(std::string filepath)
 * \brief Plot the wound segmentation convex hull.
 * \param filepath The path to the file with wound segmentation poses data.
 *---------------------------------------------------------------------------------------*/
void WoundSegmentation::plotWoundConvexHull(std::string filepath)
{
  std::vector<cv::Point> hull = getWoundConvexHullPoints(filepath);
  std::vector<cv::Point> points = getPointsList(filepath);

  cv::Mat img(imageWidth, imageHeight, CV_8UC3);

  if (points.size() > 0)
  {
    img = cv::Scalar::all(0);

    // Plot points on image
    for(size_t i = 1; i < points.size(); i++)
    {
      cv::Point pt = points[i];
      cv::circle(img, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
    }

    if (hull.size() > 0)
    {
      // Plot hull on image
      std::vector<std::vector<cv::Point>> contours;
      contours.push_back(hull);
      cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
    }

    // Show the image
    cv::imshow("hull", img);

    cv::waitKey(30);
  }
}

}