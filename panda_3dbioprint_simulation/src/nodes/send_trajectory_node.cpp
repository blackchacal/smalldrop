#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Dense>

void line_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b, bool cyclic = false);
void polynomial_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b, bool cyclic = false);
void arc_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose center, float radius, bool cyclic = false);
void currentPoseCallback(geometry_msgs::PoseConstPtr msg);

geometry_msgs::Pose initial_pose;
bool has_pose = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_trajectory_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/desired_pose", 10);
  ros::Subscriber sub = nh.subscribe("/cartesian_impedance_controller/current_pose", 10, currentPoseCallback);

  geometry_msgs::Pose final_pose;
  final_pose.position.x = initial_pose.position.x;
  final_pose.position.y = initial_pose.position.y;
  final_pose.position.z = initial_pose.position.z - 0.2;
  final_pose.orientation.x = initial_pose.orientation.x;
  final_pose.orientation.y = initial_pose.orientation.y;
  final_pose.orientation.z = initial_pose.orientation.z;
  final_pose.orientation.w = initial_pose.orientation.w;

  // Process current pose callback
  ros::Rate r(10);
  while (ros::ok() && !has_pose)
  {
    ros::spinOnce();
    r.sleep();
  }
  
  int freq = 100; // 100 Hz
  int ttime = 5; // Trajectory duration in seconds

  // line_trajectory(pub, ttime, freq, initial_pose, final_pose, true);

  // Move to edge of arc first
  float radius = 0.2;
  geometry_msgs::Pose arc_start, center;
  arc_start.position.x = initial_pose.position.x + radius;
  arc_start.position.z = initial_pose.position.y;
  arc_start.position.z = initial_pose.position.z - 0.1;
  arc_start.orientation = initial_pose.orientation;

  center.position.x = initial_pose.position.x;
  center.position.z = initial_pose.position.y;
  center.position.z = initial_pose.position.z - 0.1;
  center.orientation = initial_pose.orientation;

  line_trajectory(pub, 2, freq, initial_pose, center);
  line_trajectory(pub, 2, freq, center, arc_start);
  arc_trajectory(pub, ttime, freq, center, radius, true);
  
  return 0;
}

void line_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b, bool cyclic)
{
  static tf::TransformBroadcaster br;

  // Calculate direction vector
  Eigen::Vector3d u;
  Eigen::Vector3d pA;
  pA << pose_a.position.x, pose_a.position.y, pose_a.position.z;
  Eigen::Vector3d pB;
  pB << pose_b.position.x, pose_b.position.y, pose_b.position.z;

  u = pB - pA;

  // t varies from 0 to 1
  float t = 0;
  ros::Rate r(frequency);
  while (ros::ok() && t <= 1)
  {
    Eigen::Vector3d p;
    p = pA + t * u;

    // Broadcast tf of trajectory points
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(p.x(), p.y(), p.z()) );
    tf::Quaternion q(pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z, pose_a.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "trajectory"));

    // Send trajectory to robot
    geometry_msgs::Pose msg;
    msg.position.x = p.x();
    msg.position.y = p.y();
    msg.position.z = p.z();
    msg.orientation.x = pose_a.orientation.x;
    msg.orientation.y = pose_a.orientation.y;
    msg.orientation.z = pose_a.orientation.z;
    msg.orientation.w = pose_a.orientation.w;
    pub.publish(msg);

    t += 1/(duration*frequency);

    ros::spinOnce();
    r.sleep();
  }

  if (cyclic) {
    line_trajectory(pub, duration, frequency, pose_b, pose_a, cyclic);
  }
}

void polynomial_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose pose_a, geometry_msgs::Pose pose_b, bool cyclic)
{

}

void arc_trajectory(ros::Publisher pub, double duration, float frequency, geometry_msgs::Pose center, float radius, bool cyclic)
{
  static tf::TransformBroadcaster br;

  // For now let's assume the arc is made on the plane z = a
  // Initial position is considered center

  float t = 0;
  ros::Rate r(frequency);
  while (ros::ok() && t <= duration*frequency)
  {
    Eigen::Vector3d p;
    p << radius*cos((2*M_PI/(duration*frequency))*t)+center.position.x,
         radius*sin((2*M_PI/(duration*frequency))*t)+center.position.y,
         center.position.z;

    // Broadcast tf of trajectory points
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(p.x(), p.y(), p.z()) );
    tf::Quaternion q(center.orientation.x, center.orientation.y, center.orientation.z, center.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "trajectory"));

    // Send trajectory to robot
    geometry_msgs::Pose msg;
    msg.position.x = p.x();
    msg.position.y = p.y();
    msg.position.z = p.z();
    msg.orientation.x = center.orientation.x;
    msg.orientation.y = center.orientation.y;
    msg.orientation.z = center.orientation.z;
    msg.orientation.w = center.orientation.w;
    pub.publish(msg);

    t += 1;

    ros::spinOnce();
    r.sleep();
  }

  // TODO: Cyclic not working correctly
  if (cyclic) {
    arc_trajectory(pub, duration, frequency, center, radius, cyclic);
  }
}

void currentPoseCallback(geometry_msgs::PoseConstPtr msg)
{
  initial_pose.position.x = msg->position.x;
  initial_pose.position.y = msg->position.y;
  initial_pose.position.z = msg->position.z;
  initial_pose.orientation.x = msg->orientation.x;
  initial_pose.orientation.y = msg->orientation.y;
  initial_pose.orientation.z = msg->orientation.z;
  initial_pose.orientation.w = msg->orientation.w;
  has_pose = true;
}