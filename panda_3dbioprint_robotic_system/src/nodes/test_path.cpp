#include <ros/ros.h>
#include <panda_3dbioprint_robotic_system/toolpath_planner.h>
#include <panda_3dbioprint_vision_system/wound_segmentation.h>

using namespace vision_system;
using namespace robotic_system;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_path");
  ros::NodeHandle nh;

  unsigned int imageWidth = 400;
  unsigned int imageHeight = 400;

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points_convexhull.dat";
  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0.4, 0.7, -0.15, 0.15);
  ToolpathPlanner pl = ToolpathPlanner(imageWidth, imageHeight, 0.4, 0.7, -0.15, 0.15);
  std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);
  // std::vector<cv::Point> path = pl.genToolpathZigZag(contour, 5, IMAGE_AXIS::Y);
  std::vector<cv::Point> path = pl.genToolpathParallelLines(contour, 5, IMAGE_AXIS::Y);
  // std::vector<cv::Point> path = pl.genToolpathGrid(contour, 10, 5);
  std::vector<geometry_msgs::Pose> new_path = pl.convPathPoint2Pose(path, 0.4);

  std::cout << "Size path points: " << path.size() << std::endl;
  std::cout << "Size path pose: " << new_path.size() << std::endl;

  for (size_t k = 0; k < new_path.size(); k++)
  {
    std::cout << "Pose: " << new_path[k].position.x << " " << new_path[k].position.y \
    << " " << new_path[k].position.z << " " << new_path[k].orientation.x << " " << new_path[k].orientation.y \
    << " " << new_path[k].orientation.z << " " << new_path[k].orientation.w << std::endl;
  }
  

  cv::Mat img(imageWidth, imageHeight, CV_8UC3);
  img = cv::Scalar::all(0);

  if (contour.size() > 0)
  {
    // Plot hull on image
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);
    cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
  }

  if (path.size() > 1)
  {
    for (size_t i = 1; i < path.size(); i++)
    {
      // cv::circle(img, path[i], 2, cv::Scalar(0,0,255), 1, cv::LINE_AA);
      cv::line(img, path[i-1], path[i], cv::Scalar(0,0,255), 1, cv::LINE_AA);
    }
  }

  cv::Rect bounding_box = cv::boundingRect(contour);
  bounding_box -= cv::Point(10, 10); // Add padding to bounding box
  bounding_box += cv::Size(20, 20); // Add padding to bounding box
  cv::Mat crop = img(bounding_box);
  cv::Mat final_img(imageWidth, imageWidth*((double)bounding_box.width/(double)bounding_box.height), CV_8UC3);
  final_img = cv::Scalar::all(0);
  cv::resize(crop, final_img, final_img.size(), 0, 0, cv::INTER_CUBIC);
  std::cout << "Bounding box: " << bounding_box.width << " " << bounding_box.height << " " << ((double)bounding_box.width/(double)bounding_box.height) << std::endl;

  cv::imshow("hull", final_img);
  cv::waitKey(30);

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}