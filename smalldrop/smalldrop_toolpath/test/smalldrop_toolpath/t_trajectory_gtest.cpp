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
#include <smalldrop_state/exceptions.h>
#include <iostream>

using namespace smalldrop::smalldrop_toolpath;
using namespace smalldrop::smalldrop_state;

class TrajectoryTest : public ::testing::Test
{
protected:
  pose_t pose_1, pose_2, pose_3, pose_4, pose_5, pose_6;
  pose_t initial_pose, final_pose, center;
  std::shared_ptr<Line> line;
  std::shared_ptr<Circle> circle;
  std::shared_ptr<CircularSpiral> circular_spiral;

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
    pose_2.position.y = 5;
    pose_2.position.z = 0.4;
    pose_2.orientation.x = 1;
    pose_2.orientation.y = 0;
    pose_2.orientation.z = 0;
    pose_2.orientation.w = 0;

    line.reset(new Line(pose_1, pose_2));
    circle.reset(new Circle(pose_1, pose_2, 2, 50, PATH_PLANE::XY));
    circular_spiral.reset(new CircularSpiral(pose_1, 2.0, 0.3, 8, 50, PATH_PLANE::XY));
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
  
  EXPECT_EQ(traj1.length(), line->length());
  EXPECT_EQ(traj2.length(), circle->length());
  EXPECT_EQ(traj3.length(), circular_spiral->length());
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
  
  EXPECT_EQ(traj1.duration(), duration);
  EXPECT_EQ(traj2.duration(), duration);
  EXPECT_EQ(traj3.duration(), duration);

  duration = 25;
  pl.setDuration(duration);

  Trajectory traj4(pl.plan(*line));
  Trajectory traj5(pl.plan(*circle));
  Trajectory traj6(pl.plan(*circular_spiral));
  
  EXPECT_EQ(traj4.duration(), duration);
  EXPECT_EQ(traj5.duration(), duration);
  EXPECT_EQ(traj6.duration(), duration);
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
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
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
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
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
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
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
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
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
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
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
  
  EXPECT_EQ(traj1.length(), 0);
  EXPECT_EQ(traj2.length(), 0);
  EXPECT_EQ(traj3.length(), 0);
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
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
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
  
  EXPECT_TRUE(traj1.empty());
  EXPECT_TRUE(traj2.empty());
  EXPECT_TRUE(traj3.empty());
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

  frequency = 1000;
  pl.setFrequency(frequency);
  Trajectory traj4(pl.plan(*line));
  Trajectory traj5(pl.plan(*circle));
  Trajectory traj6(pl.plan(*circular_spiral));
  
  EXPECT_LT(traj1.poses().size(), traj4.poses().size());
  EXPECT_LT(traj2.poses().size(), traj5.poses().size());
  EXPECT_LT(traj3.poses().size(), traj6.poses().size());
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
  
  frequency = 1000;
  pl.setFrequency(frequency);
  Trajectory traj4(pl.plan(*line));
  Trajectory traj5(pl.plan(*circle));
  Trajectory traj6(pl.plan(*circular_spiral));

  EXPECT_EQ(traj1.length(), traj4.length());
  EXPECT_EQ(traj2.length(), traj5.length());
  EXPECT_EQ(traj3.length(), traj6.length());
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

  pl.setMaxSpeed(line->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj1(pl.plan(*line)));
  pl.setMaxSpeed(circle->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj2(pl.plan(*circle)));
  pl.setMaxSpeed(circular_spiral->length()/duration + 1);
  EXPECT_NO_THROW(Trajectory traj3(pl.plan(*circular_spiral)));
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
  std::vector<Path> paths = {l, c};
  Trajectory t(pl.plan(paths));

  EXPECT_FALSE(t.empty());
  EXPECT_NE(t.length(), 0);
  EXPECT_EQ(t.length(), l.length()+c.length());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
