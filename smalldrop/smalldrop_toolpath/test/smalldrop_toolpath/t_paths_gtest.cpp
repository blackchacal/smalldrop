// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file t_paths_gtest.cpp
 * \brief Test file for Path derived classes.
 */

#include <gtest/gtest.h>
#include <smalldrop_toolpath/line.h>
#include <smalldrop_toolpath/circle.h>
#include <smalldrop_toolpath/circular_spiral.h>
#include <iostream>

using namespace smalldrop::smalldrop_toolpath;

class PathsTest : public ::testing::Test
{
protected:
  geometry_msgs::Pose pose_1, pose_2, pose_3, pose_4, pose_5, pose_6;

  void SetUp() override
  {
    pose_1.position.x = 0;
    pose_1.position.y = 0;
    pose_1.position.z = 0;
    pose_1.orientation.x = 1;
    pose_1.orientation.y = 0;
    pose_1.orientation.z = 0;
    pose_1.orientation.w = 0;

    pose_2.position.x = 2;
    pose_2.position.y = 0;
    pose_2.position.z = 0;
    pose_2.orientation.x = 1;
    pose_2.orientation.y = 0;
    pose_2.orientation.z = 0;
    pose_2.orientation.w = 0;

    pose_3.position.x = 3;
    pose_3.position.y = 0;
    pose_3.position.z = 0;
    pose_3.orientation.x = 1;
    pose_3.orientation.y = 0;
    pose_3.orientation.z = 0;
    pose_3.orientation.w = 0;

    pose_4.position.x = 4;
    pose_4.position.y = 0;
    pose_4.position.z = 0;
    pose_4.orientation.x = 1;
    pose_4.orientation.y = 0;
    pose_4.orientation.z = 0;
    pose_4.orientation.w = 0;

    pose_5.position.x = 2;
    pose_5.position.y = 3;
    pose_5.position.z = 5;
    pose_5.orientation.x = 1;
    pose_5.orientation.y = 0;
    pose_5.orientation.z = 0;
    pose_5.orientation.w = 0;

    pose_6.position.x = 10;
    pose_6.position.y = 5;
    pose_6.position.z = 1;
    pose_6.orientation.x = 1;
    pose_6.orientation.y = 0;
    pose_6.orientation.z = 0;
    pose_6.orientation.w = 0;
  }

  void TearDown() override
  {
  }
};

TEST_F(PathsTest, lineLengthIsCorrect)
{
  Line l1(pose_1, pose_2);
  Line l2(pose_1, pose_3);
  Line l3(pose_1, pose_4);
  Line l4(pose_5, pose_6);

  EXPECT_EQ(l1.length(), 2);
  EXPECT_EQ(l2.length(), 3);
  EXPECT_EQ(l3.length(), 4);
  EXPECT_NEAR(l4.length(), 9.16515, 0.00001);
}

TEST_F(PathsTest, linePosesEqualInitialAndFinalPoses)
{
  Line l1(pose_1, pose_2);
  Line l2(pose_1, pose_3);

  poses_t path_1 = l1.poses();
  poses_t path_2 = l2.poses();

  EXPECT_EQ(path_1[0].position.x, pose_1.position.x);
  EXPECT_EQ(path_1[0].position.y, pose_1.position.y);
  EXPECT_EQ(path_1[0].position.z, pose_1.position.z);
  EXPECT_EQ(path_1[0].orientation.x, pose_1.orientation.x);
  EXPECT_EQ(path_1[0].orientation.y, pose_1.orientation.y);
  EXPECT_EQ(path_1[0].orientation.z, pose_1.orientation.z);
  EXPECT_EQ(path_1[0].orientation.w, pose_1.orientation.w);

  EXPECT_EQ(path_1[1].position.x, pose_2.position.x);
  EXPECT_EQ(path_1[1].position.y, pose_2.position.y);
  EXPECT_EQ(path_1[1].position.z, pose_2.position.z);
  EXPECT_EQ(path_1[1].orientation.x, pose_2.orientation.x);
  EXPECT_EQ(path_1[1].orientation.y, pose_2.orientation.y);
  EXPECT_EQ(path_1[1].orientation.z, pose_2.orientation.z);
  EXPECT_EQ(path_1[1].orientation.w, pose_2.orientation.w);

  EXPECT_EQ(path_2[0].position.x, pose_1.position.x);
  EXPECT_EQ(path_2[0].position.y, pose_1.position.y);
  EXPECT_EQ(path_2[0].position.z, pose_1.position.z);
  EXPECT_EQ(path_2[0].orientation.x, pose_1.orientation.x);
  EXPECT_EQ(path_2[0].orientation.y, pose_1.orientation.y);
  EXPECT_EQ(path_2[0].orientation.z, pose_1.orientation.z);
  EXPECT_EQ(path_2[0].orientation.w, pose_1.orientation.w);

  EXPECT_EQ(path_2[1].position.x, pose_3.position.x);
  EXPECT_EQ(path_2[1].position.y, pose_3.position.y);
  EXPECT_EQ(path_2[1].position.z, pose_3.position.z);
  EXPECT_EQ(path_2[1].orientation.x, pose_3.orientation.x);
  EXPECT_EQ(path_2[1].orientation.y, pose_3.orientation.y);
  EXPECT_EQ(path_2[1].orientation.z, pose_3.orientation.z);
  EXPECT_EQ(path_2[1].orientation.w, pose_3.orientation.w);
}

TEST_F(PathsTest, circleLengthIsCorrect)
{
  double radius = 1;

  // Needs at least 15 points for the length to be correct to +- 0.1
  unsigned int n_points = 15;
  Circle c1(pose_1, pose_2, radius, n_points);
  EXPECT_NEAR(c1.length(), 2*M_PI*radius, 0.1);
  
  // Needs at least 35 points for the length to be correct to +- 0.01
  n_points = 35;
  Circle c2(pose_1, pose_2, radius, n_points, PATH_PLANE::YZ);
  EXPECT_NEAR(c2.length(), 2*M_PI*radius, 0.01);

  // Needs at least 105 points for the length to be correct to +- 0.001
  n_points = 105;
  Circle c3(pose_1, pose_2, radius, n_points);
  EXPECT_NEAR(c3.length(), 2*M_PI*radius, 0.001);
}

TEST_F(PathsTest, circlePathHasCorrectNumberOfPoses)
{
  double radius = 0.5;

  // The number of path points should be n_points+1, because the
  // last point should be equal to the first to close the circle.

  unsigned int n_points = 5;
  Circle c1(pose_1, pose_2, radius, n_points, PATH_PLANE::XY);
  EXPECT_EQ(c1.poses().size(), n_points+1);

  n_points = 32;
  Circle c2(pose_1, pose_2, radius, n_points);
  EXPECT_EQ(c2.poses().size(), n_points+1);

  n_points = 200;
  Circle c3(pose_1, pose_2, radius, n_points);
  EXPECT_EQ(c3.poses().size(), n_points+1);

  n_points = 500;
  Circle c4(pose_1, pose_2, radius, n_points, PATH_PLANE::XZ);
  EXPECT_EQ(c4.poses().size(), n_points+1);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
