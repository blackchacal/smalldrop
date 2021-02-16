// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file wound_segmentation_comanip_convex_hull.cpp
 * \brief Defines class for co-manipulation wound segmentation convex hull algorithm.
 */

#include <smalldrop_segmentation/wound_segmentation_comanip_convex_hull.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

namespace smalldrop
{
namespace smalldrop_segmentation
{
/*****************************************************************************************
 * Public methods & constructors/destructors
 *****************************************************************************************/

/**
 * \copybrief WSegmentCoManipConvexHull::WSegmentCoManipConvexHull(const std::string filepath, const img_wsp_calibration_t calibration_data)
 */
WSegmentCoManipConvexHull::WSegmentCoManipConvexHull(const std::string filepath, const img_wsp_calibration_t calibration_data)
  : WSegmentCoManip(filepath, calibration_data)
{
  // Get save poses and convert them to points
  poses_t poses = loadWoundSegmentationPoses();
  points_t points;

  // Create point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width  = poses.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  double min_x = 100;
  double max_x = -100;
  double min_y = 100;
  double max_y = -100;
  double min_z = 100;
  double max_z = -100;
  double offset_x = 0.005;
  double offset_y = 0.005;
  double offset_z = 0.005;

  for(size_t i = 0; i < poses.size(); i++)
  {
    point_t pt = convPoseToPoint(poses[i]);
    points.push_back(pt);

    // // Add points to point cloud
    // cloud->points[i].x = poses[i].position.x;
    // cloud->points[i].y = poses[i].position.y;
    // cloud->points[i].z = poses[i].position.z;

    // Find x, y and z min and max values
    if (poses[i].position.x < min_x)
      min_x = poses[i].position.x;
    if (poses[i].position.x > max_x)
      max_x = poses[i].position.x;

    if (poses[i].position.y < min_y)
      min_y = poses[i].position.y;
    if (poses[i].position.y > max_y)
      max_y = poses[i].position.y;

    if (poses[i].position.z < min_z)
      min_z = poses[i].position.z;
    if (poses[i].position.z > max_z)
      max_z = poses[i].position.z;
  }

  // Add points to point cloud
  int i = 0;
  for (double y = min_y; y <= max_y; y += offset_y)
  {
    for (double x = min_x, z = min_z; x <= max_x; x += offset_x)
    {
      cloud->points[i].x = x;
      cloud->points[i].y = y;
      cloud->points[i].z = z;

      if (x <= (max_x+min_x)/2.0)
        z += (max_z-min_z) / ((max_x-min_x)/offset_x);
      else
        z -= (max_z-min_z) / ((max_x-min_x)/offset_x);
      i++;
    }
  }

  // Use convex hull algorithm to get the contour from the given points/poses
  poses_t hull_poses;
  points_t hull_points;
  // points_t hull_region_points;
  // poses_t hull_region_poses;
  if (points.size() > 0) 
  {
    std::vector<int> hull_positions;
    cv::convexHull(cv::Mat(points), hull_positions, true);
    cv::convexHull(cv::Mat(points), hull_points, true);

    for (size_t i = 0; i < hull_positions.size(); i++)
      hull_poses.push_back(poses[hull_positions[i]]);

    // for (size_t j = 0; j < points.size(); j++)
    // {
    //   if (cv::pointPolygonTest(hull_points, points[j], true) > 0)
    //   {
    //     hull_region_points.push_back(points[j]);
    //     hull_region_poses.push_back(poses[j]);
    //   }
    // }
  }

  contours_.push_back(hull_points);
  poses_contours_.push_back(hull_poses);
  // contours_region_.push_back(hull_region_points);
  // poses_contours_region_.push_back(hull_region_poses);

  // Get Contour Region points


  // Create mesh
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (20.5);
  gp3.setMaximumNearestNeighbors (200);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  pcl::io::savePLYFile("/home/rtonet/mesh_comanip.ply", triangles);
}

}  // namespace smalldrop_segmentation

}  // namespace smalldrop