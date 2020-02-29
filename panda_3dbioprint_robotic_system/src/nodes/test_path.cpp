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
  // std::vector<cv::Point> path = pl.genToolpathParallelLines(contour, 2, IMAGE_AXIS::Y);
  std::vector<cv::Point> path = pl.genToolpathGrid(contour, 10, 5);

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

  cv::imshow("hull", img);
  cv::waitKey(30);

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}