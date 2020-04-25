// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_wound_segmentation_comanip_convex_hull_gtest.cpp
 * \brief Test file for WSegmentCoManipConvexHull class.
 */

#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <smalldrop_segmentation/wound_segmentation_comanip_convex_hull.h>
#include <ros/package.h>

using namespace smalldrop::smalldrop_segmentation;

class WSegmentCoManipConvexHullTest : public ::testing::Test
{
protected:
  unsigned int image_width = 800;
  unsigned int image_height = 800;
  double wsp_x_min = 0.0;
  double wsp_x_max = 1.0;
  double wsp_y_min = -0.5;
  double wsp_y_max = 0.5;
  unsigned int total_data_points_1 = 4;
  unsigned int total_data_points_2 = 3;
  unsigned int total_data_points_3 = 4;
  unsigned int total_data_points_4 = 6;
  unsigned int total_contours = 1;
  std::string filepath_1;
  std::string filepath_2;
  std::string filepath_3;
  std::string filepath_4;

  void SetUp() override
  {
    std::stringstream path_1, path_2, path_3, path_4;
    path_1 << ros::package::getPath("smalldrop_segmentation") << "/test/test_data_1.dat";
    path_2 << ros::package::getPath("smalldrop_segmentation") << "/test/test_data_2.dat";
    path_3 << ros::package::getPath("smalldrop_segmentation") << "/test/test_data_3.dat";
    path_4 << ros::package::getPath("smalldrop_segmentation") << "/test/test_data_4.dat";
    filepath_1 = path_1.str();
    filepath_2 = path_2.str();
    filepath_3 = path_3.str();
    filepath_4 = path_4.str();
  }

  void TearDown() override
  {
  }
};

TEST_F(WSegmentCoManipConvexHullTest, getOnePosesContour)
{
  WSegmentCoManipConvexHull wseg(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  poses_t contour = wseg.getWoundSegmentationPosesContour(0);
  EXPECT_EQ(contour.size(), total_data_points_1);

  poses_t no_contour_1 = wseg.getWoundSegmentationPosesContour(1);
  EXPECT_EQ(no_contour_1.size(), 0);
  poses_t no_contour_2 = wseg.getWoundSegmentationPosesContour(2);
  EXPECT_EQ(no_contour_2.size(), 0);
  poses_t no_contour_3 = wseg.getWoundSegmentationPosesContour(10);
  EXPECT_EQ(no_contour_3.size(), 0);
}

TEST_F(WSegmentCoManipConvexHullTest, getOnePointsContour)
{
  WSegmentCoManipConvexHull wseg(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  points_t contour = wseg.getWoundSegmentationPointsContour(0);
  EXPECT_EQ(contour.size(), total_data_points_1);

  points_t no_contour_1 = wseg.getWoundSegmentationPointsContour(1);
  EXPECT_EQ(no_contour_1.size(), 0);
  points_t no_contour_2 = wseg.getWoundSegmentationPointsContour(2);
  EXPECT_EQ(no_contour_2.size(), 0);
  points_t no_contour_3 = wseg.getWoundSegmentationPointsContour(10);
  EXPECT_EQ(no_contour_3.size(), 0);
}

TEST_F(WSegmentCoManipConvexHullTest, getAllPosesContours)
{
  WSegmentCoManipConvexHull wseg(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  poses_contours_t contours = wseg.getWoundSegmentationPosesContours();
  EXPECT_EQ(contours.size(), total_contours);
  EXPECT_EQ(contours[0].size(), total_data_points_1);
}

TEST_F(WSegmentCoManipConvexHullTest, getAllPointsContours)
{
  WSegmentCoManipConvexHull wseg(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  contours_t contours = wseg.getWoundSegmentationPointsContours();
  EXPECT_EQ(contours.size(), total_contours);
  EXPECT_EQ(contours[0].size(), total_data_points_1);
}

TEST_F(WSegmentCoManipConvexHullTest, getContourArea)
{
  WSegmentCoManipConvexHull wseg_1(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);
  WSegmentCoManipConvexHull wseg_2(filepath_2, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);
  WSegmentCoManipConvexHull wseg_3(filepath_3, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  EXPECT_NEAR(wseg_1.contourArea(0), 0.04, 0.01);
  EXPECT_NEAR(wseg_2.contourArea(0), 0.03, 0.01);
  EXPECT_NEAR(wseg_3.contourArea(0), 0.06, 0.01);
}

TEST_F(WSegmentCoManipConvexHullTest, getContourPerimeter)
{
  WSegmentCoManipConvexHull wseg_1(filepath_1, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);
  WSegmentCoManipConvexHull wseg_2(filepath_2, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);
  WSegmentCoManipConvexHull wseg_3(filepath_3, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  EXPECT_NEAR(wseg_1.contourPerimeter(0), 0.8, 0.01);
  EXPECT_NEAR(wseg_2.contourPerimeter(0), 0.8325, 0.01);
  EXPECT_NEAR(wseg_3.contourPerimeter(0), 1.0, 0.01);
}

TEST_F(WSegmentCoManipConvexHullTest, contourOnlyHasConvexHull)
{
  WSegmentCoManipConvexHull wseg(filepath_4, image_width, image_height, wsp_x_min, wsp_x_max, wsp_y_min, wsp_y_max);

  poses_t poses_contour = wseg.getWoundSegmentationPosesContour(0);
  points_t points_contour = wseg.getWoundSegmentationPointsContour(0);
  EXPECT_EQ(poses_contour.size(), 4);
  EXPECT_EQ(points_contour.size(), 4);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
