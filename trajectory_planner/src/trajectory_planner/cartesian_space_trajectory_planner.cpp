#include <ros/ros.h>
#include <trajectory_planner/cartesian_space_trajectory_planner.h>
#include <cmath>

namespace trajectory_planner
{
// 3rd order polynomial with zero initial and final speeds
std::vector<std::vector<double>> CartesianSpaceTrajectoryPlanner::poly3(const std::vector<double> pose0,
                                                                        const std::vector<double> posef,
                                                                        const double t0, const double tf,
                                                                        const double t)
{
  std::vector<double> a0 = { pose0[0], pose0[1], pose0[2] };
  std::vector<double> a1 = { 0, 0, 0 };
  std::vector<double> a2 = { (3 / pow(tf - t0, 2)) * (posef[0] - pose0[0]),
                             (3 / pow(tf - t0, 2)) * (posef[1] - pose0[1]),
                             (3 / pow(tf - t0, 2)) * (posef[2] - pose0[2]) };
  std::vector<double> a3 = { -(2 / pow(tf - t0, 3)) * (posef[0] - pose0[0]),
                             -(2 / pow(tf - t0, 3)) * (posef[1] - pose0[1]),
                             -(2 / pow(tf - t0, 3)) * (posef[2] - pose0[2]) };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0[6], pose0[3], pose0[4], pose0[5]);
  Eigen::Quaterniond qf(posef[6], posef[3], posef[4], posef[5]);
  qres = q0.slerp(t, qf);

  std::vector<double> poses = { a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3),
                                a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3),
                                a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3),
                                qres.x(),
                                qres.y(),
                                qres.z(),
                                qres.w() };
  std::vector<double> velocities = { a1[0] + 2 * a2[0] * (t - t0) + 3 * a3[0] * pow(t - t0, 2),
                                     a1[1] + 2 * a2[1] * (t - t0) + 3 * a3[1] * pow(t - t0, 2),
                                     a1[2] + 2 * a2[2] * (t - t0) + 3 * a3[2] * pow(t - t0, 2),
                                     0,
                                     0,
                                     0,
                                     0 };
  std::vector<double> accelerations = {
    2 * a2[0] + 6 * a3[0] * (t - t0), 2 * a2[1] + 6 * a3[1] * (t - t0), 2 * a2[2] + 6 * a3[2] * (t - t0), 0, 0, 0, 0
  };

  std::vector<std::vector<double>> trajectory;
  trajectory.resize(3);
  trajectory[0] = poses;
  trajectory[1] = velocities;
  trajectory[2] = accelerations;
  return trajectory;
}

// 3rd order polynomial with non-zero initial and final speeds
std::vector<std::vector<double>> CartesianSpaceTrajectoryPlanner::poly3c(
    const std::vector<double> pose0, const std::vector<double> posef, const std::vector<double> velocity0,
    const std::vector<double> velocityf, const double t0, const double tf, const double t)
{
  std::vector<double> a0 = { pose0[0], pose0[1], pose0[2] };
  std::vector<double> a1 = { velocity0[0], velocity0[1], velocity0[2] };
  std::vector<double> a2 = {
    (3 / pow(tf - t0, 2)) * (posef[0] - pose0[0]) - (2 / (tf - t0)) * velocity0[0] - (1 / (tf - t0)) * velocityf[0],
    (3 / pow(tf - t0, 2)) * (posef[1] - pose0[1]) - (2 / (tf - t0)) * velocity0[1] - (1 / (tf - t0)) * velocityf[1],
    (3 / pow(tf - t0, 2)) * (posef[2] - pose0[2]) - (2 / (tf - t0)) * velocity0[2] - (1 / (tf - t0)) * velocityf[2]
  };
  std::vector<double> a3 = {
    -(2 / pow(tf - t0, 3)) * (posef[0] - pose0[0]) + (1 / pow(tf - t0, 2)) * (velocityf[0] + velocity0[0]),
    -(2 / pow(tf - t0, 3)) * (posef[1] - pose0[1]) + (1 / pow(tf - t0, 2)) * (velocityf[1] + velocity0[1]),
    -(2 / pow(tf - t0, 3)) * (posef[2] - pose0[2]) + (1 / pow(tf - t0, 2)) * (velocityf[2] + velocity0[2])
  };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0[6], pose0[3], pose0[4], pose0[5]);
  Eigen::Quaterniond qf(posef[6], posef[3], posef[4], posef[5]);
  qres = q0.slerp(t, qf);

  std::vector<double> poses = { a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3),
                                a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3),
                                a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3),
                                qres.x(),
                                qres.y(),
                                qres.z(),
                                qres.w() };
  std::vector<double> velocities = { a1[0] + 2 * a2[0] * (t - t0) + 3 * a3[0] * pow(t - t0, 2),
                                     a1[1] + 2 * a2[1] * (t - t0) + 3 * a3[1] * pow(t - t0, 2),
                                     a1[2] + 2 * a2[2] * (t - t0) + 3 * a3[2] * pow(t - t0, 2),
                                     0,
                                     0,
                                     0,
                                     0 };
  std::vector<double> accelerations = {
    2 * a2[0] + 6 * a3[0] * (t - t0), 2 * a2[1] + 6 * a3[1] * (t - t0), 2 * a2[2] + 6 * a3[2] * (t - t0), 0, 0, 0, 0
  };

  std::vector<std::vector<double>> trajectory;
  trajectory.resize(3);
  trajectory[0] = poses;
  trajectory[1] = velocities;
  trajectory[2] = accelerations;
  return trajectory;
}

// Calculate via velocity based on heuristic
double CartesianSpaceTrajectoryPlanner::via_velocity(const double pos_before, const double pos, const double pos_after,
                                                     const double t_before, const double t, const double t_after)
{
  double velocity_i, velocity_ii;

  velocity_i = (pos - pos_before) / (t - t_before);  // Mean velocity before via point
  velocity_ii = (pos_after - pos) / (t_after - t);   // Mean velocity after via point

  if ((velocity_i * velocity_ii) < 0)  // Mean velocities change signal at via point
    return 0.0;                        // Velocity at via point is zero
  else
    return 0.5 * (velocity_ii + velocity_i);  // Velocity at via point is the average of the mean velocities
}

// 3rd order polynomial with via points and velocity heuristics
std::vector<std::vector<double>> CartesianSpaceTrajectoryPlanner::poly3c_vias(
    const std::vector<std::vector<double>> poses, const std::vector<double> times, const double t)
{
  if (poses[0].size() != times.size())
  {
    ROS_ERROR("The poses vector should be the same size as times vector.");
    std::vector<std::vector<double>> trajectory;
    trajectory.resize(3);
    trajectory[0] = { -1, -1, -1 };
    trajectory[1] = { -1, -1, -1 };
    trajectory[2] = { -1, -1, -1 };
    return trajectory;
  }

  std::vector<double> velocity0;
  std::vector<double> velocityf;
  velocity0.resize(3, 0);
  velocityf.resize(3, 0);


  for (size_t i = 1; i < times.size(); i++)
    if (t >= times[i - 1] && t <= times[i])
    {
      if (i == 1)
      {
        // velocity0 is already {0, 0, 0}
        velocityf[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[0][i - 1], poses[0][i], poses[0][i + 1], times[i - 1], times[i], times[i + 1]);
        velocityf[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[1][i - 1], poses[1][i], poses[1][i + 1], times[i - 1], times[i], times[i + 1]);
        velocityf[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[2][i - 1], poses[2][i], poses[2][i + 1], times[i - 1], times[i], times[i + 1]);
      }
      else if (i == times.size() - 1)
      {
        velocity0[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[0][i - 1], poses[0][i], poses[0][i + 1], times[i - 1], times[i], times[i + 1]);
        velocity0[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[1][i - 1], poses[1][i], poses[1][i + 1], times[i - 1], times[i], times[i + 1]);
        velocity0[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[2][i - 1], poses[2][i], poses[2][i + 1], times[i - 1], times[i], times[i + 1]);
        // velocityf is already {0, 0, 0}
      }
      else
      {
        velocity0[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[0][i - 2], poses[0][i - 1], poses[0][i], times[i - 2], times[i - 1], times[i]);
        velocity0[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[1][i - 2], poses[1][i - 1], poses[1][i], times[i - 2], times[i - 1], times[i]);
        velocity0[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[2][i - 2], poses[2][i - 1], poses[2][i], times[i - 2], times[i - 1], times[i]);
    
        velocityf[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[0][i - 1], poses[0][i], poses[0][i + 1], times[i - 1], times[i], times[i + 1]);
        velocityf[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[1][i - 1], poses[1][i], poses[1][i + 1], times[i - 1], times[i], times[i + 1]);
        velocityf[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[2][i - 1], poses[2][i], poses[2][i + 1], times[i - 1], times[i], times[i + 1]);
      }

      std::vector<double> pose0 = {poses[0][i - 1], poses[1][i - 1], poses[2][i - 1]};
      std::vector<double> posef = {poses[0][i], poses[1][i], poses[2][i]};
            
      return CartesianSpaceTrajectoryPlanner::poly3c(pose0, posef, velocity0, velocityf, times[i - 1], times[i], t);
    }
}

// Linear segment with parabolic blends
std::vector<std::vector<double>> CartesianSpaceTrajectoryPlanner::lspb(const std::vector<double> pose0, const std::vector<double> posef, const double accel, const double t0,
                                                                       const double tf, const double t)
{
  if (abs(accel) < (4 * abs(posef[0] - pose0[0])) / pow(tf - t0, 2) ||
    abs(accel) < (4 * abs(posef[1] - pose0[1])) / pow(tf - t0, 2) ||
    abs(accel) < (4 * abs(posef[2] - pose0[2])) / pow(tf - t0, 2))
  {
    ROS_ERROR("Invalid acceleration! The value needs to be larger.");
    std::vector<std::vector<double>> trajectory;
    trajectory.resize(3);
    trajectory[0] = { -1, -1, -1 };
    trajectory[1] = { -1, -1, -1 };
    trajectory[2] = { -1, -1, -1 };
    return trajectory;
  }

  std::vector<double> pose, velocity, acceleration;
  pose.resize(7, 0);
  velocity.resize(7, 0);
  acceleration.resize(7, 0);
  std::vector<double> delta_tb;
  delta_tb.resize(3, 0);
  delta_tb[0] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef[0] - pose0[0]) / abs(accel));
  delta_tb[1] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef[1] - pose0[1]) / abs(accel));
  delta_tb[2] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef[2] - pose0[2]) / abs(accel));
  std::vector<double> tb;
  tb.resize(3, 0);
  tb[0] = t0 + delta_tb[0];
  tb[1] = t0 + delta_tb[1];
  tb[2] = t0 + delta_tb[2];
  std::vector<double> velocity_b;
  velocity_b.resize(3, 0);
  velocity_b[0] = accel * delta_tb[0];
  velocity_b[1] = accel * delta_tb[1];
  velocity_b[2] = accel * delta_tb[2];
  std::vector<double> poseb;
  poseb.resize(3, 0);
  poseb[0] = pose0[0] + 0.5 * accel * pow(delta_tb[0], 2);
  poseb[1] = pose0[1] + 0.5 * accel * pow(delta_tb[1], 2);
  poseb[2] = pose0[2] + 0.5 * accel * pow(delta_tb[2], 2);
  std::vector<double> posebf;
  posebf.resize(3, 0);
  posebf[0] = poseb[0] + velocity_b[0] * ((tf - t0) - 2 * delta_tb[0]);
  posebf[1] = poseb[1] + velocity_b[1] * ((tf - t0) - 2 * delta_tb[1]);
  posebf[2] = poseb[2] + velocity_b[2] * ((tf - t0) - 2 * delta_tb[2]);

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0[6], pose0[3], pose0[4], pose0[5]);
  Eigen::Quaterniond qf(posef[6], posef[3], posef[4], posef[5]);
  qres = q0.slerp(t, qf);
  pose[3] = qres.x();
  pose[4] = qres.y();
  pose[5] = qres.z();
  pose[6] = qres.w();

  for (size_t i = 0; i < 3; i++)
  {
    if (t >= t0 && t <= tb[i])
    {
      acceleration[i] = accel;
      velocity[i] = accel * (t - t0);
      pose[i] = pose0[i] + 0.5 * acceleration[i] * pow(t - t0, 2);
    }
    else if (t >= (tf - delta_tb[i]) && t <= tf + 0.0001)
    {
      acceleration[i] = -accel;
      velocity[i] = velocity_b[i] + acceleration[i] * (t - (tf - delta_tb[i]));
      pose[i] = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) + 0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
    }
    else
    {
      acceleration[i] = 0;
      velocity[i] = velocity_b[i];
      pose[i] = poseb[i] + velocity_b[i] * (t - tb[i]);
    }
  }
  
  std::vector<std::vector<double>> trajectory;
  trajectory.resize(3);
  trajectory[0] = pose;
  trajectory[1] = velocity;
  trajectory[2] = acceleration;
  return trajectory;
}

geometry_msgs::Pose CartesianSpaceTrajectoryPlanner::poly3p(const geometry_msgs::Pose pose0,
                                                            const geometry_msgs::Pose posef, const double t0,
                                                            const double tf, const double t)
{
  std::vector<double> a0 = { pose0.position.x, pose0.position.y, pose0.position.z };
  std::vector<double> a1 = { 0, 0, 0 };
  std::vector<double> a2 = { (3 / pow(tf - t0, 2)) * (posef.position.x - pose0.position.x),
                             (3 / pow(tf - t0, 2)) * (posef.position.y - pose0.position.y),
                             (3 / pow(tf - t0, 2)) * (posef.position.z - pose0.position.z) };
  std::vector<double> a3 = { -(2 / pow(tf - t0, 3)) * (posef.position.x - pose0.position.x),
                             -(2 / pow(tf - t0, 3)) * (posef.position.y - pose0.position.y),
                             -(2 / pow(tf - t0, 3)) * (posef.position.z - pose0.position.z) };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0.orientation.w, pose0.orientation.x, pose0.orientation.y, pose0.orientation.z);
  Eigen::Quaterniond qf(posef.orientation.w, posef.orientation.x, posef.orientation.y, posef.orientation.z);
  qres = q0.slerp(t, qf);

  geometry_msgs::Pose pose;
  pose.position.x = a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3);
  pose.position.y = a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3);
  pose.position.z = a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  return pose;
}

// 3rd order polynomial with non-zero initial and final velocities, that uses geometry_msgs::Pose
geometry_msgs::Pose CartesianSpaceTrajectoryPlanner::poly3pc(const geometry_msgs::Pose pose0,
                                                             const geometry_msgs::Pose posef,
                                                             const std::vector<double> velocity0,
                                                             const std::vector<double> velocityf, const double t0,
                                                             const double tf, const double t)
{
  std::vector<double> a0 = { pose0.position.x, pose0.position.y, pose0.position.z };
  std::vector<double> a1 = { velocity0[0], velocity0[1], velocity0[2] };
  std::vector<double> a2 = {
    (3 / pow(tf - t0, 2)) * (posef.position.x - pose0.position.x) - (2 / (tf - t0)) * velocity0[0] - (1 / (tf - t0)) * velocityf[0],
    (3 / pow(tf - t0, 2)) * (posef.position.y - pose0.position.y) - (2 / (tf - t0)) * velocity0[1] - (1 / (tf - t0)) * velocityf[1],
    (3 / pow(tf - t0, 2)) * (posef.position.z - pose0.position.z) - (2 / (tf - t0)) * velocity0[2] - (1 / (tf - t0)) * velocityf[2]
  };
  std::vector<double> a3 = {
    -(2 / pow(tf - t0, 3)) * (posef.position.x - pose0.position.x) + (1 / pow(tf - t0, 2)) * (velocityf[0] + velocity0[0]),
    -(2 / pow(tf - t0, 3)) * (posef.position.y - pose0.position.y) + (1 / pow(tf - t0, 2)) * (velocityf[1] + velocity0[1]),
    -(2 / pow(tf - t0, 3)) * (posef.position.z - pose0.position.z) + (1 / pow(tf - t0, 2)) * (velocityf[2] + velocity0[2])
  };

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0.orientation.w, pose0.orientation.x, pose0.orientation.y, pose0.orientation.z);
  Eigen::Quaterniond qf(posef.orientation.w, posef.orientation.x, posef.orientation.y, posef.orientation.z);
  qres = q0.slerp(t, qf);

  geometry_msgs::Pose pose;
  pose.position.x = a0[0] + a1[0] * (t - t0) + a2[0] * pow(t - t0, 2) + a3[0] * pow(t - t0, 3);
  pose.position.y = a0[1] + a1[1] * (t - t0) + a2[1] * pow(t - t0, 2) + a3[1] * pow(t - t0, 3);
  pose.position.z = a0[2] + a1[2] * (t - t0) + a2[2] * pow(t - t0, 2) + a3[2] * pow(t - t0, 3);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  return pose;
}

// 3rd order polynomial with via points and velocity heuristics, that uses geometry_msgs::Pose
geometry_msgs::Pose CartesianSpaceTrajectoryPlanner::poly3pc_vias(const std::vector<geometry_msgs::Pose> poses,
                                                      const std::vector<double> times, const double t)
{
  if (poses.size() != times.size())
  {
    ROS_ERROR("The poses vector should be the same size as times vector.");
    geometry_msgs::Pose pose;
    pose.position.x = -1;
    pose.position.y = -1;
    pose.position.z = -1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
  }

  std::vector<double> velocity0;
  std::vector<double> velocityf;
  velocity0.resize(3, 0);
  velocityf.resize(3, 0);

  for (size_t i = 1; i < times.size(); i++)
    if (t >= times[i - 1] && t <= times[i])
    {
      if (i == 1)
      {
        // velocity0 is already {0, 0, 0}
        velocityf[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1], times[i], times[i + 1]);
        velocityf[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1], times[i], times[i + 1]);
        velocityf[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1], times[i], times[i + 1]);
      }
      else if (i == times.size() - 1)
      {
        velocity0[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1], times[i], times[i + 1]);
        velocity0[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1], times[i], times[i + 1]);
        velocity0[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1], times[i], times[i + 1]);
        // velocityf is already {0, 0, 0}
      }
      else
      {
        velocity0[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 2].position.x, poses[i - 1].position.x, poses[i].position.x, times[i - 1], times[i], times[i + 1]);
        velocity0[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 2].position.y, poses[i - 1].position.y, poses[i].position.y, times[i - 1], times[i], times[i + 1]);
        velocity0[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 2].position.z, poses[i - 1].position.z, poses[i].position.z, times[i - 1], times[i], times[i + 1]);
    
        velocityf[0] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.x, poses[i].position.x, poses[i + 1].position.x, times[i - 1], times[i], times[i + 1]);
        velocityf[1] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.y, poses[i].position.y, poses[i + 1].position.y, times[i - 1], times[i], times[i + 1]);
        velocityf[2] = CartesianSpaceTrajectoryPlanner::via_velocity(poses[i - 1].position.z, poses[i].position.z, poses[i + 1].position.z, times[i - 1], times[i], times[i + 1]);
      }
            
      return CartesianSpaceTrajectoryPlanner::poly3pc(poses[i - 1], poses[i], velocity0, velocityf, times[i - 1], times[i], t);
    }
}

// Linear segment with parabolic blends, that uses geometry_msgs::Pose
geometry_msgs::Pose CartesianSpaceTrajectoryPlanner::lspbp(const geometry_msgs::Pose pose0, const geometry_msgs::Pose posef, const double accel, const double t0,
                                                           const double tf, const double t)
{
  // It is assumed that accel param is always positive

  if (accel < (4 * abs(posef.position.x - pose0.position.x)) / pow(tf - t0, 2) ||
    accel < (4 * abs(posef.position.y - pose0.position.y)) / pow(tf - t0, 2) ||
    accel < (4 * abs(posef.position.z - pose0.position.z)) / pow(tf - t0, 2))
  {
    ROS_ERROR("Invalid acceleration! The value needs to be larger.");
    geometry_msgs::Pose pose;
    pose.position.x = -1;
    pose.position.y = -1;
    pose.position.z = -1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
  }

  // Change accel sign depending on growth direction of position (eg. x0 > xf => accel0 < 0)
  std::vector<double> accel_vec;
  accel_vec.resize(3, 0);
  accel_vec[0] = (pose0.position.x > posef.position.x) ? -accel : accel;
  accel_vec[1] = (pose0.position.y > posef.position.y) ? -accel : accel;
  accel_vec[2] = (pose0.position.z > posef.position.z) ? -accel : accel;

  geometry_msgs::Pose pose;
  std::vector<double> velocity, acceleration;
  velocity.resize(3, 0);
  acceleration.resize(3, 0);
  std::vector<double> delta_tb;
  delta_tb.resize(3, 0);
  delta_tb[0] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef.position.x - pose0.position.x) / abs(accel_vec[0]));
  delta_tb[1] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef.position.y - pose0.position.y) / abs(accel_vec[1]));
  delta_tb[2] = (tf - t0) / 2 - sqrt(pow(tf - t0, 2) / 4 - abs(posef.position.z - pose0.position.z) / abs(accel_vec[2]));
  std::vector<double> tb;
  tb.resize(3, 0);
  tb[0] = t0 + delta_tb[0];
  tb[1] = t0 + delta_tb[1];
  tb[2] = t0 + delta_tb[2];
  std::vector<double> velocity_b;
  velocity_b.resize(3, 0);
  velocity_b[0] = accel_vec[0] * delta_tb[0];
  velocity_b[1] = accel_vec[1] * delta_tb[1];
  velocity_b[2] = accel_vec[2] * delta_tb[2];
  std::vector<double> poseb;
  poseb.resize(3, 0);
  poseb[0] = pose0.position.x + 0.5 * accel_vec[0] * pow(delta_tb[0], 2);
  poseb[1] = pose0.position.y + 0.5 * accel_vec[1] * pow(delta_tb[1], 2);
  poseb[2] = pose0.position.z + 0.5 * accel_vec[2] * pow(delta_tb[2], 2);
  std::vector<double> posebf;
  posebf.resize(3, 0);
  posebf[0] = poseb[0] + velocity_b[0] * ((tf - t0) - 2 * delta_tb[0]);
  posebf[1] = poseb[1] + velocity_b[1] * ((tf - t0) - 2 * delta_tb[1]);
  posebf[2] = poseb[2] + velocity_b[2] * ((tf - t0) - 2 * delta_tb[2]);

  // Apply slerp
  Eigen::Quaterniond qres;
  Eigen::Quaterniond q0(pose0.orientation.w, pose0.orientation.x, pose0.orientation.y, pose0.orientation.z);
  Eigen::Quaterniond qf(posef.orientation.w, posef.orientation.x, posef.orientation.y, posef.orientation.z);
  qres = q0.slerp(t, qf);
  pose.orientation.x = qres.x();
  pose.orientation.y = qres.y();
  pose.orientation.z = qres.z();
  pose.orientation.w = qres.w();

  for (size_t i = 0; i < 3; i++)
  {
    if (t >= t0 && t <= tb[i])
    {
      acceleration[i] = accel_vec[i];
      velocity[i] = accel_vec[i] * (t - t0);
      switch (i)
      {
        case 1:
          pose.position.y = pose0.position.y + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
        case 2:
          pose.position.z = pose0.position.z + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
        default:
          pose.position.x = pose0.position.x + 0.5 * acceleration[i] * pow(t - t0, 2);
          break;
      }
    }
    else if (t >= (tf - delta_tb[i]) && t <= tf + 0.0001)
    {
      acceleration[i] = -accel_vec[i];
      velocity[i] = velocity_b[i] + acceleration[i] * (t - (tf - delta_tb[i]));
      switch (i)
      {
        case 1:
          pose.position.y = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) + 0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
        case 2:
          pose.position.z = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) + 0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
        default:
          pose.position.x = posebf[i] + velocity_b[i] * (t - (tf - delta_tb[i])) + 0.5 * acceleration[i] * pow(t - (tf - delta_tb[i]), 2);
          break;
      }
    }
    else
    {
      acceleration[i] = 0;
      velocity[i] = velocity_b[i];
      switch (i)
      {
        case 1:
          pose.position.y = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
        case 2:
          pose.position.z = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
        default:
          pose.position.x = poseb[i] + velocity_b[i] * (t - tb[i]);
          break;
      }
    }
  }
  
  return pose;
}

}  // namespace trajectory_planner