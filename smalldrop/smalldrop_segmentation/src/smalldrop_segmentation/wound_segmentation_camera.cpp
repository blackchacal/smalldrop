// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera.cpp
 * \brief Defines base class for camera wound segmentation algorithms.
 */

#include <smalldrop_segmentation/wound_segmentation_camera.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCam::WSegmentCam(const img_wsp_calibration_t calibration_data)
 */
WSegmentCam::WSegmentCam(const img_wsp_calibration_t calibration_data)
{
  image_width_ = calibration_data.img_width;
  image_height_ = calibration_data.img_height;
  wsp_x_min_limit_ = calibration_data.wsp_x_min;
  wsp_x_max_limit_ = calibration_data.wsp_x_max;
  wsp_y_min_limit_ = calibration_data.wsp_y_min;
  wsp_y_max_limit_ = calibration_data.wsp_y_max;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContour() const
 */
poses_t WSegmentCam::getWoundSegmentationPosesContour(const unsigned int contour_idx) const
{
  poses_t empty_contour;
  if (contour_idx < poses_contours_.size())
    return poses_contours_[contour_idx];
  else
    return empty_contour;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContour() const
 */
points_t WSegmentCam::getWoundSegmentationPointsContour(const unsigned int contour_idx) const
{
  points_t empty_contour;
  if (contour_idx < contours_.size())
    return contours_[contour_idx];
  else
    return empty_contour;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContours() const
 */
poses_contours_t WSegmentCam::getWoundSegmentationPosesContours() const
{
  return poses_contours_;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContours() const
 */
contours_t WSegmentCam::getWoundSegmentationPointsContours() const
{
  return contours_;
}

/**
 * \copydoc WSegmentCam::contourArea(const unsigned int contour_idx) const
 */
double WSegmentCam::contourArea(const unsigned int contour_idx) const
{
  if (contour_idx < contours_.size() && contours_[contour_idx].size() >= 3)
    return cv::contourArea(contours_[contour_idx]) * convPxSq2MeterSq();
  else
    return 0;
}

/**
 * \copydoc WSegmentCam::contourPerimeter(const unsigned int contour_idx) const
 */
double WSegmentCam::contourPerimeter(const unsigned int contour_idx) const
{
  std::vector<double> px_dim = convPx2Meter();
  if (contour_idx < contours_.size() && contours_[contour_idx].size() > 1)
    return cv::arcLength(contours_[contour_idx], true) * px_dim[0];
  else
    return 0;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContourRegion() const
 */
poses_t WSegmentCam::getWoundSegmentationPosesContourRegion(const unsigned int contour_idx) const
{
  poses_t empty_contour;
  if (contour_idx < poses_contours_region_.size())
    return poses_contours_region_[contour_idx];
  else
    return empty_contour;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContourRegion() const
 */
points_t WSegmentCam::getWoundSegmentationPointsContourRegion(const unsigned int contour_idx) const
{
  points_t empty_contour;
  if (contour_idx < contours_region_.size())
    return contours_region_[contour_idx];
  else
    return empty_contour;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPosesContoursRegion() const
 */
poses_contours_t WSegmentCam::getWoundSegmentationPosesContoursRegion() const
{
  return poses_contours_region_;
}

/**
 * \copybrief WSegmentCam::getWoundSegmentationPointsContoursRegion() const
 */
contours_t WSegmentCam::getWoundSegmentationPointsContoursRegion() const
{
  return contours_region_;
}

/*****************************************************************************************
 * Protected methods
 *****************************************************************************************/

/**
 * \copybrief WSegmentCam::getContoursRegion()
 */
void WSegmentCam::getContoursRegion()
{
  for (size_t i = 0; i < contours_.size(); i++)
  {
    cv::Rect box = boundingRect(contours_[i]);
    points_t contour_area_pts;
    for (size_t x = box.x; x < box.x+box.width; x++)
    {
      for (size_t y = box.y; y < box.y+box.height; y++)
      {
        if (cv::pointPolygonTest(contours_[i], cv::Point(x,y), false) > 0)
          contour_area_pts.push_back(cv::Point(x,y));
      }
    }
    contours_region_.push_back(contour_area_pts);
  }
}

/**
 * \copybrief WSegmentCam::getPosesContours(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
 */
void WSegmentCam::getPosesContours(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
{
  for (size_t i = 0; i < contours_.size(); i++)
  {
    poses_t poses_contour;
    for (size_t j = 0; j < contours_[i].size(); j++)
    {
      int x, y;
      double delta = 0.005;
      x = contours_[i][j].x;
      y = contours_[i][j].y;

      double xr = (wsp_x_max_limit_ - wsp_x_min_limit_) / image_width_ * x + wsp_x_min_limit_;
      double yr = (wsp_y_max_limit_ - wsp_y_min_limit_) / image_height_ * y + wsp_y_min_limit_;
      
      Eigen::Vector4d pt_cam_contour, pt_base_contour;
      pt_cam_contour << xr, yr, 0.43, 1;
      pt_base_contour = transform * pt_cam_contour;

      for (size_t k = 0; k < point_cloud.points.size(); k++)
      {
        pcl::PointXYZ pt = point_cloud.points[k];
        Eigen::Vector4d pt_cam, pt_base;
        pt_cam << pt.x, pt.y, pt.z, 1;
        pt_base = transform * pt_cam;
        if (pt_base(0) <= pt_base_contour(0) + delta && pt_base(0) >= pt_base_contour(0) - delta && pt_base(1) <= pt_base_contour(1) + delta && pt_base(1) >= pt_base_contour(1) - delta)
        {
          pose_t pose;
          pose.position.x = pt_base(0);
          pose.position.y = pt_base(1);
          pose.position.z = pt_base(2);
          pose.orientation.w = 1.0;

          poses_contour.push_back(pose);
        }
      }
    }
    poses_contours_.push_back(poses_contour);
  }
}

/**
 * \copybrief WSegmentCam::getPosesContoursRegion(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
 */
void WSegmentCam::getPosesContoursRegion(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
{
  marker_pub = nh.advertise<visualization_msgs::Marker>("/smalldrop/rviz/wound_segmentation_markers_cloud", 10);

  for (size_t i = 0; i < contours_region_.size(); i++)
  {
    poses_t poses_contour_region;
    std::vector<geometry_msgs::Point> points;
    std::vector<geometry_msgs::Point> contour_points;
    int contour_marker_id = 0;
    int contour_marker2_id = 0;
    for (size_t j = 0; j < contours_region_[i].size(); j++)
    {
      int x, y;
      double delta = 0.001;
      x = contours_region_[i][j].x;
      y = contours_region_[i][j].y;

      double xr = (wsp_x_max_limit_ - wsp_x_min_limit_) / image_width_ * x + wsp_x_min_limit_;
      double yr = (wsp_y_max_limit_ - wsp_y_min_limit_) / image_height_ * y + wsp_y_min_limit_;
      
      Eigen::Vector4d pt_cam_contour, pt_base_contour;
      pt_cam_contour << xr, yr, 0.43, 1;
      pt_base_contour = transform * pt_cam_contour;

      geometry_msgs::Point ptc;
      ptc.x = pt_base_contour(0);
      ptc.y = pt_base_contour(1);
      ptc.z = pt_base_contour(2);
      contour_points.push_back(ptc);

      for (size_t k = 0; k < point_cloud.points.size(); k++)
      {
        pcl::PointXYZ pt = point_cloud.points[k];
        Eigen::Vector4d pt_cam, pt_base;
        pt_cam << pt.x, pt.y, pt.z, 1;
        pt_base = transform * pt_cam;

        if (pt_base(0) <= pt_base_contour(0) + delta && pt_base(0) >= pt_base_contour(0) - delta && pt_base(1) <= pt_base_contour(1) + delta && pt_base(1) >= pt_base_contour(1) - delta)
        {
          pose_t pose;
          geometry_msgs::Point pt;
          pt.x = pt_base(0);
          pt.y = pt_base(1);
          pt.z = pt_base(2);

          pose.position.x = pt_base(0);
          pose.position.y = pt_base(1);
          pose.position.z = pt_base(2);
          pose.orientation.w = 1.0;
          poses_contour_region.push_back(pose);
          points.push_back(pt);

          break;
        }
      }
    }
    poses_contours_region_.push_back(poses_contour_region);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time();
    marker.ns = "wound_segmentation_contour_cloud";
    marker.id = contour_marker_id++;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.scale.x = 0.003;
    marker.scale.y = 0.003;
    marker.scale.z = 0.003;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker_pub.publish(marker);

    // visualization_msgs::Marker marker2;
    // marker2.header.frame_id = "panda_link0";
    // marker2.header.stamp = ros::Time();
    // marker2.ns = "wound_segmentation_contour_cloud_region";
    // marker2.id = contour_marker2_id++;
    // marker2.type = visualization_msgs::Marker::SPHERE_LIST;
    // marker2.action = visualization_msgs::Marker::ADD;
    // marker2.points = contour_points;
    // marker2.scale.x = 0.003;
    // marker2.scale.y = 0.003;
    // marker2.scale.z = 0.003;
    // marker2.pose.position.x = 0;
    // marker2.pose.position.y = 0;
    // marker2.pose.position.z = 0;
    // marker2.pose.orientation.w = 1.0;
    // marker2.color.r = 0;
    // marker2.color.g = 1.0;
    // marker2.color.b = 0;
    // marker2.color.a = 1.0;
    // marker_pub.publish(marker2);
  }
}

/**
 * \copybrief WSegmentCam::filterPointCloud(const PointCloud& point_cloud) const
 */
PointCloud WSegmentCam::filterPointCloud(const PointCloud& point_cloud) const
{
  // Create the filtering object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  *cloud_ptr = point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits (wsp_x_min_limit_, wsp_x_max_limit_);
  pass.filter(*cloud_filtered_ptr);
  
  pass.setInputCloud(cloud_filtered_ptr);
  pass.setFilterFieldName("y");
  pass.setFilterLimits (wsp_y_min_limit_, wsp_y_max_limit_);
  pass.filter(*cloud_filtered_ptr);

  pass.setInputCloud(cloud_filtered_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits (0.3, 0.75);
  pass.filter(*cloud_filtered_ptr);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_filtered_ptr);
  sor.setLeafSize (0.005f, 0.005f, 0.01f);
  sor.filter (*cloud_filtered_ptr);

  return *cloud_filtered_ptr;
}

/*****************************************************************************************
 * Private methods
 *****************************************************************************************/

/**
 * \copybrief WSegmentCam::convPx2Meter() const
 */
std::vector<double> WSegmentCam::convPx2Meter() const
{
  double px_w = (wsp_y_max_limit_ - wsp_y_min_limit_) / image_width_;
  double px_h = (wsp_x_max_limit_ - wsp_x_min_limit_) / image_height_;
  std::vector<double> px_dim;
  px_dim.push_back(px_w);
  px_dim.push_back(px_h);

  return px_dim;
}

/**
 * \copybrief WSegmentCam::convPxSq2MeterSq() const
 */
double WSegmentCam::convPxSq2MeterSq() const
{
  std::vector<double> px_dim = convPx2Meter();
  return px_dim[0]*px_dim[1];
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop