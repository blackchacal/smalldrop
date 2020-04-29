// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_toolpaths_gtest.cpp
 * \brief Test file for ToolPath derived classes.
 */

#include <gtest/gtest.h>
#include <smalldrop_segmentation/wound_segmentation_comanip.h>
#include <smalldrop_toolpath/zigzag.h>
#include <smalldrop_toolpath/parallel_lines.h>
#include <smalldrop_toolpath/grid.h>
#include <ros/package.h>
#include <iostream>
#include <sstream>

using namespace smalldrop::smalldrop_segmentation;
using namespace smalldrop::smalldrop_toolpath;

class ToolPathsTest : public ::testing::Test
{
protected:
  img_wsp_calibration_t calibration_data = {
    .img_width = 800,
    .img_height = 800,
    .wsp_x_min = 0.0,
    .wsp_x_max = 1.0,
    .wsp_y_min = -0.5,
    .wsp_y_max = 0.5
  };
  points_t contour;
  point_t point_1, point_2, point_3, point_4;

  void SetUp() override
  {
    point_1.x = 10;
    point_1.y = 10;
    point_2.x = 210;
    point_2.y = 10;
    point_3.x = 210;
    point_3.y = 110;
    point_4.x = 10;
    point_4.y = 110;

    contour.push_back(point_1);
    contour.push_back(point_2);
    contour.push_back(point_3);
    contour.push_back(point_4);
  }

  void TearDown() override
  {
  }
};

TEST_F(ToolPathsTest, toolPathActionsAreValid)
{
  // Create toolpath for given contour
  unsigned int offset = 5;
  unsigned int offset_x = 5;
  unsigned int offset_y = 5;
  IMAGE_AXIS axis = IMAGE_AXIS::X;
  double pose_z = 0;
  ZigZag zz(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl(contour, offset, axis, pose_z, calibration_data);
  Grid gd(contour, offset_x, offset_y, axis, pose_z, calibration_data);

  // For ZigZag & ParallelLines toolpaths the actions should be
  // START, for first pose
  // STOP, for last pose
  // CONTINUE, for all intermediate poses
  path_actions_t actions_zz = zz.actions();
  for (unsigned int i = 0; i < actions_zz.size(); i++)
  {
    if (i == 0)
      EXPECT_EQ(actions_zz[i], PRINT_ACTION::START);
    else if (i == actions_zz.size()-1)
      EXPECT_EQ(actions_zz[i], PRINT_ACTION::STOP);
    else
      EXPECT_EQ(actions_zz[i], PRINT_ACTION::CONTINUE);
  }
  path_actions_t actions_pl = pl.actions();
  for (unsigned int i = 0; i < actions_pl.size(); i++)
  {
    if (i == 0)
      EXPECT_EQ(actions_pl[i], PRINT_ACTION::START);
    else if (i == actions_pl.size()-1)
      EXPECT_EQ(actions_pl[i], PRINT_ACTION::STOP);
    else
      EXPECT_EQ(actions_pl[i], PRINT_ACTION::CONTINUE);
  }

  // For Grid toolpath the actions should be
  // START, for first pose
  // STOP, for last pose
  // CONTINUE, for all intermediate poses, except two poses when pattern changes
  // from vertical to horizontal or vice-versa.
  // One pose is STOP and the other START
  path_actions_t actions_gd = gd.actions();
  int start_cnt = 0;
  int stop_cnt = 0;
  for (unsigned int i = 0; i < actions_gd.size(); i++)
  {
    if (i == 0)
      EXPECT_EQ(actions_gd[i], PRINT_ACTION::START);
    else if (i == actions_gd.size()-1)
      EXPECT_EQ(actions_gd[i], PRINT_ACTION::STOP);
    else
    {
      if (actions_gd[i] == PRINT_ACTION::START)
        start_cnt++;
      if (actions_gd[i] == PRINT_ACTION::STOP)
        stop_cnt++;
    }
  }

  EXPECT_EQ(start_cnt, 1);
  EXPECT_EQ(stop_cnt, 1);
}

TEST_F(ToolPathsTest, ifContourIsEmptyToolPathLengthIsZero)
{
  // Create toolpath for given contour
  unsigned int offset = 5;
  unsigned int offset_x = 5;
  unsigned int offset_y = 5;
  IMAGE_AXIS axis = IMAGE_AXIS::X;
  double pose_z = 0;
  points_t empty_contour;
  ZigZag zz(empty_contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl(empty_contour, offset, axis, pose_z, calibration_data);
  Grid gd(empty_contour, offset_x, offset_y, axis, pose_z, calibration_data);

  EXPECT_EQ(zz.length(), 0);
  EXPECT_EQ(pl.length(), 0);
  EXPECT_EQ(gd.length(), 0);
}

TEST_F(ToolPathsTest, allPathPosesHaveSameZValue)
{
  // Create toolpath for given contour
  unsigned int offset = 5;
  unsigned int offset_x = 5;
  unsigned int offset_y = 5;
  IMAGE_AXIS axis = IMAGE_AXIS::X;
  double pose_z = 10;
  ZigZag zz(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl(contour, offset, axis, pose_z, calibration_data);
  Grid gd(contour, offset_x, offset_y, axis, pose_z, calibration_data);

  poses_t poses_zz = zz.poses();
  poses_t poses_pl = pl.poses();
  poses_t poses_gd = gd.poses();
  for (unsigned int i = 0; i < poses_zz.size(); i++)
    EXPECT_EQ(poses_zz[i].position.z, pose_z);
  for (unsigned int i = 0; i < poses_pl.size(); i++)
    EXPECT_EQ(poses_pl[i].position.z, pose_z);
  for (unsigned int i = 0; i < poses_gd.size(); i++)
    EXPECT_EQ(poses_gd[i].position.z, pose_z);
}

TEST_F(ToolPathsTest, numberOfPosesDecreasesWithIncreasingOffset)
{
  // Create toolpath for given contour
  IMAGE_AXIS axis = IMAGE_AXIS::X;
  double pose_z = 10;

  unsigned int offset = 5;
  unsigned int offset_x = 5;
  unsigned int offset_y = 5;
  ZigZag zz_1(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl_1(contour, offset, axis, pose_z, calibration_data);
  Grid gd_1(contour, offset_x, offset_y, axis, pose_z, calibration_data);
  offset = 10;
  offset_x = 10;
  offset_y = 10;
  ZigZag zz_2(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl_2(contour, offset, axis, pose_z, calibration_data);
  Grid gd_2(contour, offset_x, offset_y, axis, pose_z, calibration_data);
  offset = 20;
  offset_x = 20;
  offset_y = 20;
  ZigZag zz_3(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl_3(contour, offset, axis, pose_z, calibration_data);
  Grid gd_3(contour, offset_x, offset_y, axis, pose_z, calibration_data);
  offset = 30;
  offset_x = 30;
  offset_y = 30;
  ZigZag zz_4(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl_4(contour, offset, axis, pose_z, calibration_data);
  Grid gd_4(contour, offset_x, offset_y, axis, pose_z, calibration_data);
  offset = 50;
  offset_x = 50;
  offset_y = 50;
  ZigZag zz_5(contour, offset, axis, pose_z, calibration_data);
  ParallelLines pl_5(contour, offset, axis, pose_z, calibration_data);
  Grid gd_5(contour, offset_x, offset_y, axis, pose_z, calibration_data);

  EXPECT_GT(zz_1.poses().size(), zz_2.poses().size());
  EXPECT_GT(zz_2.poses().size(), zz_3.poses().size());
  EXPECT_GT(zz_3.poses().size(), zz_4.poses().size());
  EXPECT_GT(zz_4.poses().size(), zz_5.poses().size());

  EXPECT_GT(pl_1.poses().size(), pl_2.poses().size());
  EXPECT_GT(pl_2.poses().size(), pl_3.poses().size());
  EXPECT_GT(pl_3.poses().size(), pl_4.poses().size());
  EXPECT_GT(pl_4.poses().size(), pl_5.poses().size());

  EXPECT_GT(gd_1.poses().size(), gd_2.poses().size());
  EXPECT_GT(gd_2.poses().size(), gd_3.poses().size());
  EXPECT_GT(gd_3.poses().size(), gd_4.poses().size());
  EXPECT_GT(gd_4.poses().size(), gd_5.poses().size());

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
