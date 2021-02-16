// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file gazebo_models_to_rviz_node.cpp 
 * \brief Node to handle publishing of gazebo models to rviz as markers.
 */

#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <smalldrop_vision/camera_d415.h>
#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>
#include <smalldrop_toolpath/trajectory_planner.h>
#include <smalldrop_toolpath/zigzag.h>
#include <smalldrop_toolpath/parallel_lines.h>
#include <smalldrop_toolpath/grid.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

// ROS messages
#include <visualization_msgs/Marker.h>
#include <smalldrop_msgs/SetPathType.h>

// Libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include <Eigen/Dense>

using namespace smalldrop::smalldrop_vision;
using namespace smalldrop::smalldrop_segmentation;
using namespace smalldrop::smalldrop_toolpath;

/**
 * Global variables
 *********************************************************************************************/

std::string toolpath_image_topic = "/smalldrop/rviz/toolpath_image";
unsigned int image_width = 640;
unsigned int image_height = 480;
std::string path_type = "zig_zag";
unsigned int offset_x = 5;
unsigned int offset_y = 5;
std::string axis = "x";
unsigned int contour_marker_id = 0;
unsigned int path_marker_id = 0;
camera_topics_t camera_topics;

/**
 * Function prototypes
 *********************************************************************************************/

bool changePathType(smalldrop_msgs::SetPathType::Request &req, smalldrop_msgs::SetPathType::Response &res);
void publishWoundSegmentationAreaMarkers(ros::Publisher pub, poses_t contour);
void publishToolpathMarkers(ros::Publisher pub, poses_t path);
sensor_msgs::ImagePtr getToolpathImageForPublication(WSegmentCamBinary &wseg, const Eigen::Matrix4d& transform, img_wsp_calibration_t calibration_data);
points_t getPath(points_t contour, poses_t contour_region, const Eigen::Matrix4d& transform, unsigned int offset_x, unsigned int offset_y, std::string axis, double pose_z, img_wsp_calibration_t calibration_data);
poses_t getPathPoses(points_t contour, poses_t contour_region, const Eigen::Matrix4d& transform, unsigned int offset_x, unsigned int offset_y, std::string axis, double pose_z, img_wsp_calibration_t calibration_data);
void plotText(cv::Mat img, points_t contour, point_t pt);
cv::Mat zoomOnContour(cv::Mat img_src, points_t contour, unsigned int box_padding);
void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform);

/**
 * Main
 *********************************************************************************************/

int main( int argc, char** argv )
{
  // TODO: 
  // - Use SystemConfig data
  // - Check point addition by file size change or use topic to get points data
  // - Use SystemState to get publishers

  ros::init(argc, argv, "toolpath_viewer_camera");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_toolpath = it.advertise(toolpath_image_topic, 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/smalldrop/rviz/wound_segmentation_markers", 10);
  ros::ServiceServer service = nh.advertiseService("change_path_type", changePathType);

  // double distance = 0.702;
  // double distance = 0.549;
  double distance = 0.688;
  double wsp_w = 0.890 * distance; // 0.885 Depth FOV VGA (radians), from camera datasheet (page 35)
  // double wsp_w = 0.838 * distance; 
  double wsp_h = 0.75 * wsp_w;
  img_wsp_calibration_t calibration_data = {
    .img_width = image_width,
    .img_height = image_height,
    .wsp_x_min = -wsp_w/2,
    .wsp_x_max = wsp_w/2,
    .wsp_y_min = -wsp_h/2,
    .wsp_y_max = wsp_h/2
  };

  ROS_INFO("Calibration data, w: %d, h: %d, xmin: %f, xmax: %f, ymin: %f, ymax: %f\n", calibration_data.img_width, \
  calibration_data.img_height, \
  calibration_data.wsp_x_min, \
  calibration_data.wsp_x_max, \
  calibration_data.wsp_y_min, \
  calibration_data.wsp_y_max);

  camera_topics.rgb_info_topic = "/smalldrop/vision/camera/color/camera_info";
  camera_topics.rgb_image_topic = "/smalldrop/vision/camera/color/image_raw";
  // camera_topics.ir1_info_topic = "/smalldrop/vision/camera/ir/camera_info";
  // camera_topics.ir1_image_topic = "/smalldrop/vision/camera/ir/image_raw";
  // camera_topics.ir2_info_topic = "/smalldrop/vision/camera/ir2/camera_info";
  // camera_topics.ir2_image_topic = "/smalldrop/vision/camera/ir2/image_raw";
  camera_topics.ir1_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir1_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.ir2_info_topic = "/smalldrop/vision/camera/infra1/camera_info";
  camera_topics.ir2_image_topic = "/smalldrop/vision/camera/infra1/image_raw";
  camera_topics.depth_info_topic = "/smalldrop/vision/camera/depth/camera_info";
  camera_topics.depth_image_topic = "/smalldrop/vision/camera/depth/image_raw";
  camera_topics.rgb_pcloud_topic = "/smalldrop/vision/camera/depth/color/points";

  tf::TransformListener tf_listener;
  Eigen::Matrix4d transform;
  transform.setIdentity();

  CameraD415 cam(true, camera_topics);
  cam.turnOn();

  int t = 0;
  ros::Rate r(10);
  while (ros::ok())
  {
    try
    {
      getTransformToBase(tf_listener, transform);
      WSegmentCamBinary wseg(cam.getRGBImage(), cam.getDepthImage(), cam.getPointCloud(), transform, calibration_data);

      // marker_pub = nh.advertise<visualization_msgs::Marker>("/smalldrop/rviz/wound_segmentation_markers_cloud", 10);
      // const PointCloud& pcloud = cam.getPointCloud();
      // std::vector<geometry_msgs::Point> points;

      // // ROS_INFO("point cloud width: %d, height: %d", pcloud.width, pcloud.height);

      // for (size_t i = 0; i < pcloud.points.size(); i+=10)
      // {
      //   pcl::PointXYZ pt = pcloud.points[i];
      //   geometry_msgs::Point gpt;
      //   gpt.x = pt.x;
      //   gpt.y = pt.y;
      //   gpt.z = pt.z;
      //   points.push_back(gpt);
      // }
      // if (pcloud.width > 0)
      // {
      //   visualization_msgs::Marker marker;
      //   marker.header.frame_id = "panda_link0";
      //   marker.header.stamp = ros::Time();
      //   marker.ns = "wound_segmentation_contour_cloud";
      //   marker.id = contour_marker_id++;
      //   marker.type = visualization_msgs::Marker::SPHERE_LIST;
      //   marker.action = visualization_msgs::Marker::ADD;
      //   marker.points = points;
      //   marker.scale.x = 0.003;
      //   marker.scale.y = 0.003;
      //   marker.scale.z = 0.003;
      //   marker.pose.position.x = 0;
      //   marker.pose.position.y = 0;
      //   marker.pose.position.z = 0;
      //   marker.pose.orientation.w = 1.0;
      //   marker.color.r = 1.0;
      //   marker.color.g = 1.0;
      //   marker.color.b = 0;
      //   marker.color.a = 1.0;
      //   marker_pub.publish(marker);
      // }

      sensor_msgs::ImagePtr msg_toolpath = getToolpathImageForPublication(wseg, transform, calibration_data);
      pub_toolpath.publish(msg_toolpath);
      
      // Draw wound segmentation contour markers
      // poses_t poses_contour = wseg.getWoundSegmentationPosesContour(0);
      // publishWoundSegmentationAreaMarkers(marker_pub, poses_contour);

      // Draw wound filling toolpath markers
      // points_t contour = wseg.getWoundSegmentationPointsContour(0);
      
      // Only plot the path if the contour is a closed surface
      // if (contour.size() > 2)
      // {
      //   poses_t toolpath = getPathPoses(contour, wseg.getWoundSegmentationPosesContourRegion(0), transform, offset_x, offset_y, axis, 0, calibration_data);
      //   publishToolpathMarkers(marker_pub, toolpath);
      // }

      // if (t == 10)
      // {
      //   visualization_msgs::Marker marker_contour;
      //   marker_contour.header.frame_id = "panda_link0";
      //   marker_contour.header.stamp = ros::Time();
      //   marker_contour.ns = "wound_segmentation_contour";
      //   marker_contour.action = visualization_msgs::Marker::DELETEALL;
      //   marker_pub.publish(marker_contour);
        
      //   visualization_msgs::Marker marker_toolpath;
      //   marker_toolpath.header.frame_id = "panda_link0";
      //   marker_toolpath.header.stamp = ros::Time();
      //   marker_toolpath.ns = "wound_filling_path";
      //   marker_toolpath.action = visualization_msgs::Marker::DELETEALL;
      //   marker_pub.publish(marker_toolpath);

      //   contour_marker_id = 0;
      //   path_marker_id = 0;
      //   t = 0;
      // }
      // t++;

      ros::spinOnce();
      r.sleep();
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("Error creating segmentation image: %s", e.what());
      break;
    }
  }

  cam.turnOff();
  
  return 0;
}

/**
 * General functions & callbacks
 *********************************************************************************************/

/**
 * \brief Service callback to change the path parameters
 */
bool changePathType(smalldrop_msgs::SetPathType::Request &req, smalldrop_msgs::SetPathType::Response &res)
{
  path_type = req.path_type;
  offset_x = req.offset_x;
  offset_y = req.offset_y;
  axis = req.axis;
  res.ack = true;
  return true;
}

void publishWoundSegmentationAreaMarkers(ros::Publisher pub, poses_t contour)
{
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
        marker.ns = "wound_segmentation_contour";
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
    marker.ns = "wound_segmentation_contour";
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

void publishToolpathMarkers(ros::Publisher pub, poses_t path)
{
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
        p1.z = path[i].position.z;
        p2.x = path[i+1].position.x;
        p2.y = path[i+1].position.y;
        p2.z = path[i+1].position.z;
        points.push_back(p1);
        points.push_back(p2);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "panda_link0";
        marker.header.stamp = ros::Time();
        marker.ns = "wound_filling_path";
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

/**
 * Prepare toolpath image to be published for rviz
 */
sensor_msgs::ImagePtr getToolpathImageForPublication(WSegmentCamBinary &wseg, const Eigen::Matrix4d& transform, img_wsp_calibration_t calibration_data)
{
  try
  {
    points_t path;
    points_t contour = wseg.getWoundSegmentationPointsContour(0);

    cv::Mat img(image_width, image_height, CV_8UC3);
    img = cv::Scalar::all(0);

    // Only plot the path if the contour is a closed surface
    if (contour.size() > 2)
      path = getPath(contour, wseg.getWoundSegmentationPosesContourRegion(0), transform, offset_x, offset_y, axis, 0, calibration_data);
    else
    {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      return msg;
    }

    // // Plot points on image
    // for(size_t i = 0; i < contour.size(); i++)
    //   cv::circle(img, contour[i], 2, cv::Scalar(0, 255, 255), cv::FILLED, cv::LINE_AA);

    if (contour.size() > 0)
    {
      // Plot hull on image
      contours_t contours;
      contours.push_back(contour);
      cv::drawContours(img, contours, -1, cv::Scalar(0,255,0), 1, cv::LINE_AA);
    }

    if (path.size() > 1)
      // Plot the wound filling path
      for (size_t i = 1; i < path.size(); i++)
        cv::line(img, path[i-1], path[i], cv::Scalar(0,0,255), 1, cv::LINE_AA);

    // // Plot the coordinates of the contour points on image
    // // for(size_t i = 0; i < contour.size(); i++) 
    // //   plotText(img, contour, contour[i]);

    cv::Mat final_img(img);
    final_img = img;

    if (contour.size() > 2)
      // Crop image to bounding box and rescale it
      final_img = zoomOnContour(img, contour, 10);

    cv::Mat rsize(image_width, image_height, CV_8UC3);
    rsize = cv::Scalar::all(0);
    cv::resize(final_img, rsize, cv::Size(image_width, image_height), 0, 0, cv::InterpolationFlags::INTER_CUBIC);

    cv::Mat gaussBlur;
    cv::GaussianBlur(rsize, gaussBlur, cv::Size(0, 0), 3);
    cv::addWeighted(rsize, 2.0, gaussBlur, -1.0, 0, rsize);

    // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
    // cv::imshow("Display window", rsize);                   // Show our image inside it.
    // // cv::imshow("Display window", final_img);                   // Show our image inside it.

    // cv::waitKey(0);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_img).toImageMsg();

    return msg;
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Error creating toolpath image: %s", e.what());
  }
}

/**
 * \brief Generates the tool path based on configuration parameters
 */
points_t getPath(points_t contour, poses_t contour_region, const Eigen::Matrix4d& transform, unsigned int offset_x, unsigned int offset_y, std::string axis, double pose_z, img_wsp_calibration_t calibration_data)
{
  points_t path;

  if (path_type.compare("parallel_lines") == 0)
  {
    if (axis.compare("x") == 0)
    {
      ParallelLines parallel(contour, offset_x, IMAGE_AXIS::X, pose_z, calibration_data);
      path = parallel.points();
    }
    else
    {
      ParallelLines parallel(contour, offset_y, IMAGE_AXIS::Y, pose_z, calibration_data);
      path = parallel.points();
    }
  }
  else if (path_type.compare("grid") == 0)
  {
    Grid grid(contour, offset_x, offset_y, pose_z, calibration_data);
    path = grid.points();
  }
  else // zig_zag
  {
    if (axis.compare("x") == 0)
    {
      // ZigZag zigzag(contour, offset_x, IMAGE_AXIS::X, pose_z, calibration_data);
      ZigZag zigzag(contour, contour_region, transform, offset_x, IMAGE_AXIS::X, calibration_data);
      path = zigzag.points();
    }
    else
    {
      // ZigZag zigzag(contour, offset_y, IMAGE_AXIS::Y, pose_z, calibration_data);
      ZigZag zigzag(contour, contour_region, transform, offset_y, IMAGE_AXIS::Y, calibration_data);
      path = zigzag.points();
    }
  }

  return path;
}

/**
 * \brief Generates the tool path based on configuration parameters
 */
poses_t getPathPoses(points_t contour, poses_t contour_region, const Eigen::Matrix4d& transform, unsigned int offset_x, unsigned int offset_y, std::string axis, double pose_z, img_wsp_calibration_t calibration_data)
{
  poses_t poses;

  if (path_type.compare("parallel_lines") == 0)
  {
    if (axis.compare("x") == 0)
    {
      ParallelLines parallel(contour, offset_x, IMAGE_AXIS::X, pose_z, calibration_data);
      poses = parallel.poses();
    }
    else
    {
      ParallelLines parallel(contour, offset_y, IMAGE_AXIS::Y, pose_z, calibration_data);
      poses = parallel.poses();
    }
  }
  else if (path_type.compare("grid") == 0)
  {
    Grid grid(contour, offset_x, offset_y, pose_z, calibration_data);
    poses = grid.poses();
  }
  else // zig_zag
  {
    if (axis.compare("x") == 0)
    {
      // ZigZag zigzag(contour, offset_x, IMAGE_AXIS::X, pose_z, calibration_data);
      ZigZag zigzag(contour, contour_region, transform, offset_x, IMAGE_AXIS::X, calibration_data);
      poses = zigzag.poses();
    }
    else
    {
      // ZigZag zigzag(contour, offset_y, IMAGE_AXIS::Y, pose_z, calibration_data);
      ZigZag zigzag(contour, contour_region, transform, offset_x, IMAGE_AXIS::X, calibration_data);
      poses = zigzag.poses();
    }
  }

  return poses;
}

/**
 * \brief Plot the wound segmentation points coordinates on the image
 */
void plotText(cv::Mat img, points_t contour, point_t pt)
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

  point_t txt_pos;
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
cv::Mat zoomOnContour(cv::Mat img_src, points_t contour, unsigned int box_padding)
{
  // Crop image on contour
  cv::Rect bounding_box = cv::boundingRect(contour);
  bounding_box -= cv::Point(box_padding, box_padding); // Add padding to bounding box
  bounding_box += cv::Size(2*box_padding, 2*box_padding); // Add padding to bounding box
  if (bounding_box.x < 0) 
    bounding_box.x = 0;
  if (bounding_box.y < 0) 
    bounding_box.y = 0;
  if (bounding_box.width > image_width) 
    bounding_box.width = image_width;
  if (bounding_box.height > image_height) 
    bounding_box.height = image_height;
    
  cv::Mat crop = img_src(bounding_box);
  return crop;
}

void getTransformToBase(tf::TransformListener& tf_listener, Eigen::Matrix4d& transform)
{
  tf::StampedTransform transformStamped;
  try {
    tf_listener.lookupTransform("panda_link0", "camera_depth_optical_frame", ros::Time(0), transformStamped);
    
    Eigen::Vector4d t_vector(transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1);
    transform.col(3) << transformStamped.getOrigin().x(), transformStamped.getOrigin().y(), transformStamped.getOrigin().z(), 1;
    transform.col(0) << transformStamped.getBasis().getColumn(0)[0], transformStamped.getBasis().getColumn(0)[1], transformStamped.getBasis().getColumn(0)[2], 0;
    transform.col(1) << transformStamped.getBasis().getColumn(1)[0], transformStamped.getBasis().getColumn(1)[1], transformStamped.getBasis().getColumn(1)[2], 0;
    transform.col(2) << transformStamped.getBasis().getColumn(2)[0], transformStamped.getBasis().getColumn(2)[1], transformStamped.getBasis().getColumn(2)[2], 0;

    // std::cout << transform << std::endl;
  } catch (tf2::TransformException &ex) {
    return;
  }
}