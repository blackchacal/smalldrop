#include <ros/ros.h>
#include <panda_3dbioprint_robotic_system/toolpath_planner.h>
#include <panda_3dbioprint_robotic_system/SetPathType.h>
#include <image_transport/image_transport.h>
#include <sstream>

using namespace vision_system;
using namespace robotic_system;

std::string toolpath_image_topic = "/panda_3dbioprint_robotic_system/toolpath_image";
unsigned int imageWidth = 400;
unsigned int imageHeight = 400;
std::string path_type = "zig_zag";
unsigned int offset_x = 5;
unsigned int offset_y = 5;
std::string axis = "x";

/**
 * \brief Service callback to change the path parameters
 */
bool changePathType(panda_3dbioprint_robotic_system::SetPathType::Request &req,
                    panda_3dbioprint_robotic_system::SetPathType::Response &res)

{
  path_type = req.path_type;
  offset_x = req.offset_x;
  offset_y = req.offset_y;
  axis = req.axis;
  res.ack = true;
  return true;
}

/**
 * \brief Generates the tool path based on configuration parameters
 */
std::vector<cv::Point> getPath(ToolpathPlanner pl, std::vector<cv::Point> contour, unsigned int offset_x, unsigned int offset_y, std::string axis)
{
  std::vector<cv::Point> path;

  if (path_type.compare("parallel_lines") == 0)
  {
    if (axis.compare("x") == 0)
      path = pl.genToolpathParallelLines(contour, offset_x, IMAGE_AXIS::X);
    else
      path = pl.genToolpathParallelLines(contour, offset_y, IMAGE_AXIS::Y);
  }
  else if (path_type.compare("grid") == 0)
  {
    path = pl.genToolpathGrid(contour, offset_x, offset_y);
  }
  else // zig_zag
  {
    if (axis.compare("x") == 0)
      path = pl.genToolpathZigZag(contour, offset_x, IMAGE_AXIS::X);
    else
      path = pl.genToolpathZigZag(contour, offset_y, IMAGE_AXIS::Y);
  }

  return path;
}

/**
 * \brief Plot the wound segmentation points coordinates on the image
 */
void plotText(cv::Mat img, std::vector<cv::Point> contour, cv::Point pt)
{
  bool top = false, right = false, bottom = false, left = false;
  std::ostringstream txt;
  txt << "(" << pt.x << "," << pt.y << ")";

  // Check if the text is inside contour
  if (cv::pointPolygonTest(contour, cv::Point(pt.x,pt.y-3), false) > 0)
    top = true;
  if (cv::pointPolygonTest(contour, cv::Point(pt.x+3,pt.y), false) > 0)
    right = true;
  if (cv::pointPolygonTest(contour, cv::Point(pt.x,pt.y+3), false) > 0)
    bottom = true;
  if (cv::pointPolygonTest(contour, cv::Point(pt.x-3,pt.y), false) > 0)
    left = true;

  cv::Point txt_pos;
  txt_pos.x = pt.x + 2;
  txt_pos.y = pt.y - 2;
  
  if (top && right)
  {
    txt_pos.x = pt.x - 40;
    txt_pos.y = pt.y + 7;
  }
  else if (bottom && left)
  {
    txt_pos.x = pt.x + 2;
    txt_pos.y = pt.y - 2;
  }
  else if (top)
  {
    txt_pos.y = pt.y + 7;
  }
  else if (right)
  {
    txt_pos.x = pt.x - 40;
  }
  else if (left)
  {
    txt_pos.x = pt.x + 2;
  }
  
  cv::putText(img, txt.str(), txt_pos, cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(255,255,255), 1, cv::LINE_AA);
}

/**
 * \brief Zoom on the contour bonding box with some padding
 */
cv::Mat zoomOnContour(cv::Mat img_src, std::vector<cv::Point> contour, unsigned int box_padding)
{
  // Crop image on contour
  cv::Rect bounding_box = cv::boundingRect(contour);
  bounding_box -= cv::Point(box_padding, box_padding); // Add padding to bounding box
  bounding_box += cv::Size(2*box_padding, 2*box_padding); // Add padding to bounding box
  if (bounding_box.x < 0) 
    bounding_box.x = 0;
  if (bounding_box.y < 0) 
    bounding_box.y = 0;
  if (bounding_box.width > imageWidth) 
    bounding_box.width = imageWidth;
  if (bounding_box.height > imageHeight) 
    bounding_box.height = imageHeight;
    
  cv::Mat crop = img_src(bounding_box);
  return crop;
}

/**
 * Prepare image to be published for rviz
 */
sensor_msgs::ImagePtr getImageForPublication(std::string filepath)
{
  std::vector<cv::Point> path;
  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0, 1, -0.5, 0.5);
  ToolpathPlanner pl = ToolpathPlanner(imageWidth, imageHeight, 0, 1, -0.5, 0.5);
  std::vector<cv::Point> points = wseg.getPointsList(filepath);
  std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);

  // Only plot the path if the contour is a closed surface
  if (contour.size() > 2)
    path = getPath(pl, contour, offset_x, offset_y, axis);

  cv::Mat img(imageWidth, imageHeight, CV_8UC3);
  img = cv::Scalar::all(0);
  
  // Plot points on image
  for(size_t i = 0; i < points.size(); i++)
    cv::circle(img, points[i], 2, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);

  if (contour.size() > 0)
  {
    // Plot hull on image
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);
    cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
  }

  if (path.size() > 1)
    // Plot the wound filling path
    for (size_t i = 1; i < path.size(); i++)
      cv::line(img, path[i-1], path[i], cv::Scalar(0,0,255), 1, cv::LINE_AA);

  // Plot the coordinates of the contour points on image
  for(size_t i = 0; i < contour.size(); i++) 
    plotText(img, contour, contour[i]);

  cv::Mat final_img(img);
  if (contour.size() > 2)
    // Crop image to bounding box and rescale it
    final_img = zoomOnContour(img, contour, 40);
  
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();

  return msg;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "toolpath_viewer_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(toolpath_image_topic, 1);
  ros::ServiceServer service = nh.advertiseService("change_path_type", changePathType);

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";

  ros::Rate r(1);
  while (ros::ok())
  {
    sensor_msgs::ImagePtr msg = getImageForPublication(filepath);
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}