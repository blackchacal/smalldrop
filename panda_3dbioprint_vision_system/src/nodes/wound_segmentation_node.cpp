#include <ros/ros.h>
#include <panda_3dbioprint_vision_system/wound_segmentation.h>
#include <image_transport/image_transport.h>

using namespace vision_system;

std::string wound_segmentation_image_topic = "/panda_3dbioprint_vision_system/wound_segmentation_image";
unsigned int imageWidth = 400;
unsigned int imageHeight = 400;

sensor_msgs::ImagePtr getImageForPublication(WoundSegmentation& wseg, std::string filepath)
{
  std::vector<cv::Point> points = wseg.getPointsList(filepath);
  std::vector<cv::Point> contour = wseg.getWoundConvexHullPoints(filepath);

  cv::Mat img(imageWidth, imageHeight, CV_8UC3);
  img = cv::Scalar::all(0);
  
  // Plot points on image
  for(size_t i = 1; i < points.size(); i++)
  {
    cv::Point pt = points[i];
    cv::circle(img, pt, 1, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
  }

  if (contour.size() > 0)
  {
    // Plot hull on image
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(contour);
    cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
  }
  
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

  return msg;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(wound_segmentation_image_topic, 1);

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";
  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0, 1, -0.5, 0.5);

  ros::Rate r(1);
  while (ros::ok())
  {
    sensor_msgs::ImagePtr msg = getImageForPublication(wseg, filepath);
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}