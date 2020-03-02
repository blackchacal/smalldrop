#include <ros/ros.h>
#include <panda_3dbioprint_robotic_system/toolpath_planner.h>
#include <panda_3dbioprint_robotic_system/SetPathType.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
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
unsigned int contour_marker_id = 100;
unsigned int path_marker_id = 500;

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
sensor_msgs::ImagePtr getImageForPublication(ToolpathPlanner &pl, WoundSegmentation &wseg, std::string filepath)
{
  std::vector<cv::Point> path;
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

void publishToolpathMarkers(ros::Publisher pub, std::vector<geometry_msgs::Pose> path)
{
  // // Erase previous markers
  // for (size_t i = 500; i <= path_marker_id; i++)
  // {
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = "panda_link0";
  //   marker.header.stamp = ros::Time();
  //   marker.ns = "panda_3dbioprint_vision_system";
  //   marker.id = i;
  //   marker.action = visualization_msgs::Marker::DELETE;
  //   pub.publish(marker);
  // }
  // path_marker_id = 500;

  if (path.size() > 1)
  {
    for (size_t i = 0; i < path.size(); i++)
    {
      if (i < path.size() - 1)
      {
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        p1.x = path[i].position.x;
        p1.y = path[i].position.y;
        p1.z = 0;
        p2.x = path[i+1].position.x;
        p2.y = path[i+1].position.y;
        p2.z = 0;
        points.push_back(p1);
        points.push_back(p2);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "panda_3dbioprint_vision_system";
        marker.id = path_marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points = points;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.003;
        marker.color.r = 1.0;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0;
        pub.publish(marker);
      }
    }
  }
}

void publishWoundSegmentationAreaMarkers(ros::Publisher pub, std::vector<geometry_msgs::Pose> contour)
{
  // // Erase previous markers
  // for (size_t i = 100; i <= contour_marker_id; i++)
  // {
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = "panda_link0";
  //   marker.header.stamp = ros::Time();
  //   marker.ns = "panda_3dbioprint_vision_system";
  //   marker.id = i;
  //   marker.action = visualization_msgs::Marker::DELETE;
  //   pub.publish(marker);
  // }
  // contour_marker_id = 100;

  if (contour.size() > 1)
  {
    for (size_t i = 0; i < contour.size(); i++)
    {
      if (i < contour.size() - 1)
      {
        std::vector<geometry_msgs::Point> points;
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        p1.x = contour[i].position.x;
        p1.y = contour[i].position.y;
        p1.z = 0;
        p2.x = contour[i+1].position.x;
        p2.y = contour[i+1].position.y;
        p2.z = 0;
        points.push_back(p1);
        points.push_back(p2);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "panda_3dbioprint_vision_system";
        marker.id = contour_marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.points = points;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.003;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0;
        marker.color.a = 1.0;
        pub.publish(marker);
      }
    }

    // Close the contour
    std::vector<geometry_msgs::Point> points;
    geometry_msgs::Point p1;
    geometry_msgs::Point p2;
    p1.x = contour.back().position.x;
    p1.y = contour.back().position.y;
    p1.z = 0;
    p2.x = contour.front().position.x;
    p2.y = contour.front().position.y;
    p2.z = 0;
    points.push_back(p1);
    points.push_back(p2);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.ns = "panda_3dbioprint_vision_system";
    marker.id = contour_marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.003;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    pub.publish(marker);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "toolpath_viewer_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(toolpath_image_topic, 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/panda_3dbioprint_vision_system/wound_segmentation_markers", 10);
  ros::ServiceServer service = nh.advertiseService("change_path_type", changePathType);

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";

  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0, 1, -0.5, 0.5);
  ToolpathPlanner pl = ToolpathPlanner(imageWidth, imageHeight, 0, 1, -0.5, 0.5);

  int t = 0;
  ros::Rate r(1);
  while (ros::ok())
  {
    sensor_msgs::ImagePtr msg = getImageForPublication(pl, wseg, filepath);
    pub.publish(msg);
    std::vector<geometry_msgs::Pose> contour_poses = wseg.getWoundConvexHullPoses(filepath);
    publishWoundSegmentationAreaMarkers(marker_pub, contour_poses);

    // std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);
    // // Only plot the path if the contour is a closed surface
    // if (contour.size() > 2)
    // {
    //   std::vector<cv::Point> path = getPath(pl, contour, offset_x, offset_y, axis);
    //   std::vector<geometry_msgs::Pose> toolpath = pl.convPathPoint2Pose(path, 0);
    //   publishToolpathMarkers(marker_pub, toolpath);
    // }

    if (t == 10)
    {
      for (size_t i = contour_marker_id; i >= 100; i--)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "panda_3dbioprint_vision_system";
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        contour_marker_id = 100;
      }
      for (size_t i = path_marker_id; i >= 500; i--)
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "panda_3dbioprint_vision_system";
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        // contour_marker_id = 100;
        path_marker_id = 500;
      }
      t = 0;
    }
    t++;

    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}