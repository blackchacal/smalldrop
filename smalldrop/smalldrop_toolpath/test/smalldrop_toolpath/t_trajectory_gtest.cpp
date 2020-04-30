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
#include <smalldrop_toolpath/trajectory.h>
#include <smalldrop_toolpath/trajectory_planner.h>
#include <smalldrop_segmentation/wound_segmentation_comanip.h>
#include <smalldrop_toolpath/zigzag.h>
#include <smalldrop_toolpath/parallel_lines.h>
#include <smalldrop_toolpath/grid.h>
#include <smalldrop_state/exceptions.h>
#include <iostream>

using namespace smalldrop::smalldrop_toolpath;
using namespace smalldrop::smalldrop_state;

class TrajectoryTest : public ::testing::Test
{
protected:
  // Vars associated with paths
  pose_t pose_1, pose_2, pose_3, pose_4, pose_5, pose_6;
  pose_t initial_pose, final_pose, center;
  std::shared_ptr<Line> line;
  std::shared_ptr<Circle> circle;
  std::shared_ptr<CircularSpiral> circular_spiral;

  // Vars associated with toolpaths
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
  std::shared_ptr<ZigZag> zigzag;
  std::shared_ptr<ParallelLines> parallel;
  std::shared_ptr<Grid> grid;

  void SetUp() override
  {
    // Paths
    pose_1.position.x = 0;
    pose_1.position.y = 0;
    pose_1.position.z = 0;
    pose_1.orientation.x = 1;
    pose_1.orientation.y = 0;
    pose_1.orientation.z = 0;
    pose_1.orientation.w = 0;

    pose_2.position.x = 2;
    pose_2.position.y = 5;
    pose_2.position.z = 0.4;
    pose_2.orientation.x = 1;
    pose_2.orientation.y = 0;
    pose_2.orientation.z = 0;
    pose_2.orientation.w = 0;

    line.reset(new Line(pose_1, pose_2));
    circle.reset(new Circle(pose_1, pose_2, 2, 50, PATH_PLANE::XY));
    circular_spiral.reset(new CircularSpiral(pose_1, 2.0, 0.3, 8, 50, PATH_PLANE::XY));

    // Toolpaths
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

    unsigned int offset = 5;
    unsigned int offset_x = 5;
    unsigned int offset_y = 5;
    double pose_z = 0;
    IMAGE_AXIS axis = IMAGE_AXIS::X;
    zigzag.reset(new ZigZag(contour, offset, axis, pose_z, calibration_data));
    parallel.reset(new ParallelLines(contour, offset, axis, pose_z, calibration_data));
    grid.reset(new Grid(contour, offset_x, offset_y, pose_z, calibration_data));
  }

  void TearDown() override
  {
  }
};

TEST_F(TrajectoryTest, trajectoryLengthIsCorrect)
{
  double duration = 10;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);

  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.length(), line->length());
  EXPECT_EQ(traj2.length(), circle->length());
  EXPECT_EQ(traj3.length(), circular_spiral->length());
  EXPECT_EQ(traj4.length(), zigzag->length());
  EXPECT_EQ(traj5.length(), parallel->length());
  EXPECT_EQ(traj6.length(), grid->length());
}

TEST_F(TrajectoryTest, trajectoryDurationIsCorrect)
{
  double duration = 10;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.duration(), duration);
  EXPECT_EQ(traj2.duration(), duration);
  EXPECT_EQ(traj3.duration(), duration);
  EXPECT_EQ(traj4.duration(), duration);
  EXPECT_EQ(traj5.duration(), duration);
  EXPECT_EQ(traj6.duration(), duration);

  duration = 25;
  pl.setDuration(duration);

  Trajectory traj7(pl.plan(*line));
  Trajectory traj8(pl.plan(*circle));
  Trajectory traj9(pl.plan(*circular_spiral));
  Trajectory traj10(pl.plan(*zigzag));
  Trajectory traj11(pl.plan(*parallel));
  Trajectory traj12(pl.plan(*grid));
  
  EXPECT_EQ(traj7.duration(), duration);
  EXPECT_EQ(traj8.duration(), duration);
  EXPECT_EQ(traj9.duration(), duration);
  EXPECT_EQ(traj10.duration(), duration);
  EXPECT_EQ(traj11.duration(), duration);
  EXPECT_EQ(traj12.duration(), duration);
}

TEST_F(TrajectoryTest, trajectoryLengthIsZeroIfDurationIsZero)
{
  double duration = 0;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
  EXPECT_EQ(traj4.length(), 0);
  EXPECT_EQ(traj5.length(), 0);
  EXPECT_EQ(traj6.length(), 0);
}

TEST_F(TrajectoryTest, trajectoryLengthIsZeroIfDurationIsNegative)
{
  double duration = -50;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
  EXPECT_EQ(traj4.length(), 0);
  EXPECT_EQ(traj5.length(), 0);
  EXPECT_EQ(traj6.length(), 0);
}

TEST_F(TrajectoryTest, trajectoryIsEmptyIfDurationIsZero)
{
  double duration = 0;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
  EXPECT_TRUE(traj4.empty());
  EXPECT_TRUE(traj5.empty());
  EXPECT_TRUE(traj6.empty());
}

TEST_F(TrajectoryTest, trajectoryIsEmptyIfDurationIsNegative)
{
  double duration = -100;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
  EXPECT_TRUE(traj4.empty());
  EXPECT_TRUE(traj5.empty());
  EXPECT_TRUE(traj6.empty());
}

TEST_F(TrajectoryTest, trajectoryLengthIsZeroIfFrequencyIsZero)
{
  double duration = 10;
  double frequency = 0;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
  EXPECT_EQ(traj4.length(), 0);
  EXPECT_EQ(traj5.length(), 0);
  EXPECT_EQ(traj6.length(), 0);
}

TEST_F(TrajectoryTest, trajectoryLengthIsZeroIfFrequencyIsNegative)
{
  double duration = 10;
  double frequency = -30;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
  EXPECT_EQ(traj4.length(), 0);
  EXPECT_EQ(traj5.length(), 0);
  EXPECT_EQ(traj6.length(), 0);
}

TEST_F(TrajectoryTest, trajectoryIsEmptyIfFrequencyIsZero)
{
  double duration = 50;
  double frequency = 0;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
  EXPECT_TRUE(traj4.empty());
  EXPECT_TRUE(traj5.empty());
  EXPECT_TRUE(traj6.empty());
}

TEST_F(TrajectoryTest, trajectoryIsEmptyIfFrequencyIsNegative)
{
  double duration = 50;
  double frequency = -10;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
  EXPECT_TRUE(traj4.empty());
  EXPECT_TRUE(traj5.empty());
  EXPECT_TRUE(traj6.empty());
}

TEST_F(TrajectoryTest, trajectorySizeIncreasesWithFrequency)
{
  double duration = 10;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));

  frequency = 1000;
  pl.setFrequency(frequency);
  Trajectory traj7(pl.plan(*line));
  Trajectory traj8(pl.plan(*circle));
  Trajectory traj9(pl.plan(*circular_spiral));
  Trajectory traj10(pl.plan(*zigzag));
  Trajectory traj11(pl.plan(*parallel));
  Trajectory traj12(pl.plan(*grid));
  
  EXPECT_LT(traj1.poses().size(), traj7.poses().size());
  EXPECT_LT(traj2.poses().size(), traj8.poses().size());
  EXPECT_LT(traj3.poses().size(), traj9.poses().size());
  EXPECT_LT(traj4.poses().size(), traj10.poses().size());
  EXPECT_LT(traj5.poses().size(), traj11.poses().size());
  EXPECT_LT(traj6.poses().size(), traj12.poses().size());
}

TEST_F(TrajectoryTest, trajectoryLengthIsConstantWithFrequency)
{
  double duration = 50;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  Trajectory traj1(pl.plan(*line));
  Trajectory traj2(pl.plan(*circle));
  Trajectory traj3(pl.plan(*circular_spiral));
  Trajectory traj4(pl.plan(*zigzag));
  Trajectory traj5(pl.plan(*parallel));
  Trajectory traj6(pl.plan(*grid));
  
  frequency = 1000;
  pl.setFrequency(frequency);
  Trajectory traj7(pl.plan(*line));
  Trajectory traj8(pl.plan(*circle));
  Trajectory traj9(pl.plan(*circular_spiral));
  Trajectory traj10(pl.plan(*zigzag));
  Trajectory traj11(pl.plan(*parallel));
  Trajectory traj12(pl.plan(*grid));

  EXPECT_EQ(traj1.length(), traj7.length());
  EXPECT_EQ(traj2.length(), traj8.length());
  EXPECT_EQ(traj3.length(), traj9.length());
  EXPECT_EQ(traj4.length(), traj10.length());
  EXPECT_EQ(traj5.length(), traj11.length());
  EXPECT_EQ(traj6.length(), traj12.length());
}

TEST_F(TrajectoryTest, throwExceptionWhenViolatingTrajectoryMaxSpeed)
{
  double duration = 0.1;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode, 5);
  EXPECT_THROW(Trajectory traj1(pl.plan(*line)), TrajectoryMaxSpeedExceededException);
  EXPECT_THROW(Trajectory traj2(pl.plan(*circle)), TrajectoryMaxSpeedExceededException);
  EXPECT_THROW(Trajectory traj3(pl.plan(*circular_spiral)), TrajectoryMaxSpeedExceededException);
  EXPECT_THROW(Trajectory traj4(pl.plan(*zigzag)), TrajectoryMaxSpeedExceededException);
  EXPECT_THROW(Trajectory traj5(pl.plan(*parallel)), TrajectoryMaxSpeedExceededException);
  EXPECT_THROW(Trajectory traj6(pl.plan(*grid)), TrajectoryMaxSpeedExceededException);

  pl.setMaxSpeed(line->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj1(pl.plan(*line)));
  pl.setMaxSpeed(circle->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj2(pl.plan(*circle)));
  pl.setMaxSpeed(circular_spiral->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj3(pl.plan(*circular_spiral)));
  pl.setMaxSpeed(zigzag->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj3(pl.plan(*zigzag)));
  pl.setMaxSpeed(parallel->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj3(pl.plan(*parallel)));
  pl.setMaxSpeed(grid->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj3(pl.plan(*grid)));
}

TEST_F(TrajectoryTest, trajectoryWithMultiplePaths)
{
  initial_pose.position.x = 0.4;
  initial_pose.position.y = 0;
  initial_pose.position.z = 0.5;
  initial_pose.orientation.x = 1;
  initial_pose.orientation.y = 0;
  initial_pose.orientation.z = 0;
  initial_pose.orientation.w = 0;

  final_pose.position.x = 0.4;
  final_pose.position.y = 0;
  final_pose.position.z = 0;
  final_pose.orientation.x = 1;
  final_pose.orientation.y = 0;
  final_pose.orientation.z = 0;
  final_pose.orientation.w = 0;

  center.position.x = 0.3;
  center.position.y = 0;
  center.position.z = 0;
  center.orientation.x = 1;
  center.orientation.y = 0;
  center.orientation.z = 0;
  center.orientation.w = 0;

  Line l(initial_pose, final_pose);
  Circle c(final_pose, center, 0.1, 10, PATH_PLANE::XY);

  double duration = 10;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  paths_t paths = {l, c};
  Trajectory t(pl.plan(paths));

  EXPECT_FALSE(t.empty());
  EXPECT_NE(t.length(), 0);
  EXPECT_EQ(t.length(), l.length()+c.length());
}

TEST_F(TrajectoryTest, trajectoryWithMultipleToolPaths)
{
  double duration = 10;
  double frequency = 100;
  PLAN_MODE mode = PLAN_MODE::POLY3;

  TrajectoryPlanner pl(duration, frequency, mode);
  toolpaths_t toolpaths = {*zigzag, *parallel, *grid};
  Trajectory t(pl.plan(toolpaths));

  EXPECT_FALSE(t.empty());
  EXPECT_NE(t.length(), 0);
  EXPECT_EQ(t.length(), zigzag->length()+parallel->length()+grid->length());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
