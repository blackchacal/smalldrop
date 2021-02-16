// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera_binarization.cpp
 * \brief Defines class for camera wound segmentation binarization algorithm.
 */

#include <smalldrop_segmentation/wound_segmentation_camera_binarization.h>
#include <visualization_msgs/Marker.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Dense>

#include <ros/package.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCamBinary::WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image&
 * depth_image, const PointCloud& point_cloud, const Eigen::Matrix4d& transform, const img_wsp_calibration_t calibration_data)
 */
WSegmentCamBinary::WSegmentCamBinary(const sensor_msgs::Image& rgb_image, const sensor_msgs::Image& depth_image,
                                     const PointCloud& point_cloud, const Eigen::Matrix4d& transform, const img_wsp_calibration_t calibration_data)
  : WSegmentCam(calibration_data)
{ 
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    if (sensor_msgs::image_encodings::isColor(rgb_image.encoding))
      cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);
    else if (rgb_image.encoding == "16UC1")
    {
      sensor_msgs::Image img;
      img.header = rgb_image.header;
      img.height = rgb_image.height;
      img.width = rgb_image.width;
      img.is_bigendian = rgb_image.is_bigendian;
      img.step = rgb_image.step;
      img.data = rgb_image.data;
      img.encoding = "mono16";

      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
      cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  try
  {
    cv::Mat rsize(calibration_data.img_height, calibration_data.img_width, CV_8UC3);
    rsize = cv::Scalar::all(0);
    cv::resize(cv_ptr->image, rsize, rsize.size());

    // Set to grayscale
    cv::Mat grey;
    cv::cvtColor(rsize, grey, CV_BGR2GRAY);

    // Binarize image
    /*
      Threshold Type
      0: Binary
      1: Binary Inverted
      2: Threshold Truncated
      3: Threshold to Zero
      4: Threshold to Zero Inverted
    */
    cv::Mat bin;
    int threshold_value = 10;
    int threshold_type = cv::THRESH_BINARY_INV;// | cv::THRESH_OTSU;
    int const max_BINARY_value = 255;
    cv::threshold(grey, bin, threshold_value, max_BINARY_value, threshold_type);

    // Get contours
    contours_t contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(rsize, contours, -1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);

    int idx = 0;
    int max_size = 0;
    for (int i = 0; i < contours.size(); i++)
    {
      if (contours[i].size() > max_size)
      {
        max_size = contours[i].size();
        idx = i;
      }
    }
    
    if (contours.size() == 0)
    {
      std::cout << "Contours size == 0!" << std::endl;
      return;
    }
    else
    {
      contours_.push_back(contours[idx]);
    }

    // cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.
    // cv::imshow("Display window", rsize);                   // Show our image inside it.

    // cv::waitKey(0);

    // Get points inside contours
    getContoursRegion();
  }
  catch(const std::exception& e)
  {
    ROS_ERROR("Error creating segmentation image: %s", e.what());
    throw;
  }

  int contour_marker_id = 0;
  if (point_cloud.width > 0)
  {
    std::stringstream path;
    path << ros::package::getPath("smalldrop_segmentation") << "/data/point_cloud_data.dat";	
    std::string filepath = path.str();

    std::fstream fh;
    fh.open(filepath, std::fstream::out);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/smalldrop/rviz/wound_segmentation_markers_cloud", 10);
    std::vector<geometry_msgs::Point> points;

    ROS_INFO("point cloud size: %d", point_cloud.points.size());
    for (size_t k = 0; k < point_cloud.points.size(); k++)
    {
      pcl::PointXYZ pt = point_cloud.points[k];
      fh << pt.x << ", " << pt.y << ", " << pt.z << "\n";

      if (k < 5)
      {
        geometry_msgs::Point ptt;
        ptt.x = pt.x;
        ptt.y = pt.y;
        ptt.z = pt.z;
        points.push_back(ptt);

      }
    } 
    fh.close();
    visualization_msgs::Marker marker;
    marker.header.frame_id = "camera_depth_optical_frame";
    // marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.ns = "wound_segmentation_contour_cloud";
    marker.id = contour_marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.scale.x = 0.011;
    marker.scale.y = 0.011;
    marker.scale.z = 0.011;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    // marker_pub.publish(marker);
    
    // Filter the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_filtered_ptr = filterPointCloud(point_cloud, transform);

    try
    {
      // Normal estimation*
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_filtered_ptr);
      n.setInputCloud(cloud_filtered_ptr);
      n.setSearchMethod(tree);
      n.setKSearch(20);
      n.compute(*normals);
      //* normals should not contain the point normals + surface curvatures

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields(*cloud_filtered_ptr, *normals, *cloud_with_normals);
      //* cloud_with_normals = cloud + normals

      // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius(0.025);

      // Set typical values for the parameters
      gp3.setMu(2.5);
      gp3.setMaximumNearestNeighbors(100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud(cloud_with_normals);
      gp3.setSearchMethod(tree2);
      gp3.reconstruct(triangles);

      // Additional vertex information
      // std::vector<int> parts = gp3.getPartIDs();
      // std::vector<int> states = gp3.getPointStates();
      
      pcl::io::savePLYFile("/home/rtonet/mesh.ply", triangles);

      // Get poses from contours
      getPosesContours(*cloud_filtered_ptr, transform);
      // Get poses inside contours
      getPosesContoursRegion(point_cloud, transform);
      // getPosesContoursRegion(*cloud_filtered_ptr, transform);
    }
    catch(const std::exception& e)
    {
      std::cout << "Point cloud dimensions: " << point_cloud.width << "x" << point_cloud.height << std::endl;
      ROS_ERROR("Error filtering or downsampling point cloud: %s", e.what());
      throw;
    }
  }
  else
  {
    ROS_WARN("No point cloud!");
  }
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop