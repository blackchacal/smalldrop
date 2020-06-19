// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this header code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_camera.h
 * \brief Declares base class for camera wound segmentation algorithms.
 */

#ifndef _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H
#define _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H

#include <ros/ros.h>

#include <smalldrop_segmentation/i_wound_segmentation.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/**
 * \class WSegmentCam
 * \brief Base class for camera wound segmentation algorithms.
 */
class WSegmentCam : public IWoundSegmentation
{
public:
  virtual ~WSegmentCam()
  {
  }

  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPosesContour() const
   */
  virtual poses_t getWoundSegmentationPosesContour(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPointsContour() const
   */
  virtual points_t getWoundSegmentationPointsContour(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPosesContours() const
   */
  virtual poses_contours_t getWoundSegmentationPosesContours() const override;

  /**
   * \copydoc IWoundSegmentation::getWoundSegmentationPointsContours() const
   */
  virtual contours_t getWoundSegmentationPointsContours() const override;

  /**
   * \copydoc IWoundSegmentation::contourArea(const unsigned int contour_idx) const
   */
  virtual double contourArea(const unsigned int contour_idx) const override;

  /**
   * \copydoc IWoundSegmentation::contourPerimeter(const unsigned int contour_idx) const
   */
  virtual double contourPerimeter(const unsigned int contour_idx) const override;

  /**
   * \fn getWoundSegmentationPosesContourRegion() const
   * \brief Returns a vector with poses inside wound segmentation contour region.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  poses_t getWoundSegmentationPosesContourRegion(const unsigned int contour_idx) const;

  /**
   * \fn getWoundSegmentationPointsContourRegion() const
   * \brief Returns a vector with points inside wound segmentation contour region.
   * 
   * \param contour_idx Index of contour to be returned.
   */
  points_t getWoundSegmentationPointsContourRegion(const unsigned int contour_idx) const;

  /**
   * \fn getWoundSegmentationPosesContoursRegion() const
   * \brief Returns a vector with detected wound segmentation contours regions, using poses.
   */
  poses_contours_t getWoundSegmentationPosesContoursRegion() const;

  /**
   * \fn getWoundSegmentationPointsContoursRegion() const
   * \brief Returns a vector with detected wound segmentation contours regions, using points.
   */
  contours_t getWoundSegmentationPointsContoursRegion() const;

protected:
  /**
   * Class members
   *****************************************************************************************/

  poses_contours_t poses_contours_;       /** \var Vector with detected wound segmentation contours using poses. */
  contours_t contours_;                   /** \var Vector with detected wound segmentation contours. */
  poses_contours_t poses_contours_region_;  /** \var Vector with poses contained by detected wound segmentation contours */
  contours_t contours_region_;              /** \var Vector with points contained by detected wound segmentation contours */

  // Robot workspace and segmentation image
  unsigned int image_width_;  /** \var Image width in px. */
  unsigned int image_height_; /** \var Image height in px. */
  double wsp_x_min_limit_;    /** \var Camera field of view minimum x limit on robot workspace. */
  double wsp_x_max_limit_;    /** \var Camera field of view maximum x limit on robot workspace. */
  double wsp_y_min_limit_;    /** \var Camera field of view minimum y limit on robot workspace. */
  double wsp_y_max_limit_;    /** \var Camera field of view maximum y limit on robot workspace. */

  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn WSegmentCam(const img_wsp_calibration_t calibration_data)
   * \brief Default constructor.
   * 
   * \param calibration_data Data for image-workspace calibration.
   */
  WSegmentCam(const img_wsp_calibration_t calibration_data);

  /**
   * \fn contours_t getContoursRegion()
   * \brief Get points inside the region defined by the contours
   */
  void getContoursRegion();

  /**
   * \fn poses_contours_t getPosesContours(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
   * \brief Get the contours poses
   * 
   * \param point_cloud Camera point cloud
   * \param transform Transformation matrix from the camera frame to the robot base frame
   */
  void getPosesContours(const PointCloud& point_cloud, const Eigen::Matrix4d& transform);

  /**
   * \fn poses_contours_t getPosesContoursRegion(const PointCloud& point_cloud, const Eigen::Matrix4d& transform)
   * \brief Get poses inside the region defined by the contours
   * 
   * \param point_cloud Camera point cloud
   * \param transform Transformation matrix from the camera frame to the robot base frame
   */
  void getPosesContoursRegion(const PointCloud& point_cloud, const Eigen::Matrix4d& transform);

  /**
   * \fn PointCloud filterPointCloud(const PointCloud& point_cloud) const
   * \brief Filter the camera generated point cloud to reduce points. It filters on x, y and z directions
   * and uses voxel grid for downsampling.
   * 
   * \param point_cloud Camera generated point cloud.
   */
  PointCloud filterPointCloud(const PointCloud& point_cloud) const;

private:
  /**
   * Class methods
   *****************************************************************************************/

  /**
   * \fn double convPx2Meter()
   * \brief Convert the pixel size to meters
   */
  std::vector<double> convPx2Meter() const;

  /**
   * \fn double convPxSq2MeterSq()
   * \brief Convert the pixel squared area to meter squared.
   */
  double convPxSq2MeterSq() const;
};

}  // namespace smalldrop_segmentation

}  // namespace smalldrop

#endif  // _SMALLDROP_WOUND_SEGMENTATION_CAMERA_H