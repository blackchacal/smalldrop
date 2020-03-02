#include <ros/ros.h>
#include <panda_3dbioprint_vision_system/wound_segmentation.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>

using namespace vision_system;

std::string wound_segmentation_image_topic = "/panda_3dbioprint_vision_system/wound_segmentation_image";
unsigned int imageWidth = 400;
unsigned int imageHeight = 400;
unsigned int marker_id = 100;

// Function declarations
void publishWoundSegmentationAreaMarkers(ros::Publisher pub, std::vector<geometry_msgs::Pose> contour);
sensor_msgs::ImagePtr getImageForPublication(WoundSegmentation& wseg, std::string filepath);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "wound_segmentation_viewer_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise(wound_segmentation_image_topic, 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/panda_3dbioprint_vision_system/wound_segmentation_markers", 10);

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points.dat";
  WoundSegmentation wseg = WoundSegmentation(imageWidth, imageHeight, 0, 1, -0.5, 0.5);

  ros::Rate r(1);
  while (ros::ok())
  {
    sensor_msgs::ImagePtr msg = getImageForPublication(wseg, filepath);
    std::vector<geometry_msgs::Pose> contour_poses = wseg.getWoundConvexHullPoses(filepath);
    publishWoundSegmentationAreaMarkers(marker_pub, contour_poses);
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}

void publishWoundSegmentationAreaMarkers(ros::Publisher pub, std::vector<geometry_msgs::Pose> contour)
{
  // Erase previous markers
  for (size_t i = 100; i <= marker_id; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.ns = "panda_3dbioprint_vision_system";
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;
    pub.publish(marker);
  }

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
        marker.id = marker_id++;
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
    marker.id = marker_id++;
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