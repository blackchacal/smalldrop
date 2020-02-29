#include <ros/ros.h>
#include <panda_3dbioprint_vision_system/wound_segmentation.h>
#include <panda_3dbioprint_vision_system/spatial_2d_processor.h>

using namespace cv;
using namespace std;
using namespace vision_system;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  unsigned int imageWidth = 400;
  unsigned int imageHeight = 400;

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points_convexhull.dat";
  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0.4, 0.7, -0.15, 0.15);
  Spatial2DProcessor p = Spatial2DProcessor(imageWidth, imageHeight, 0.4, 0.7, -0.15, 0.15);
  std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);
  //wseg.plotWoundConvexHull(filepath);

  cv::Mat img(imageWidth, imageHeight, CV_8UC3);
  img = cv::Scalar::all(0);

  if (contour.size() > 0)
  {
    // Plot hull on image
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);
    cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
  }
  cv::Rect bound = cv::boundingRect(contour);
  cv::rectangle(img, bound, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

  cv::imshow("hull", img);
  cv::waitKey();

  // double area = p.calcWoundContourArea(contour);
  // double perimeter = p.calcWoundContourPerimeter(contour);

  // std::cout << "Area: " << area << " mm^2, Perimeter: " << perimeter << " m" << std::endl;

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}