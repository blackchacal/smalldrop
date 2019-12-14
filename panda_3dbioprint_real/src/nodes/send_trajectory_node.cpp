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
  final_pose.position.x = 0.555;
  final_pose.position.y = 0.0;
  final_pose.position.z = 0.419;
  final_pose.orientation.x = 0.9239;
  final_pose.orientation.y = 0.3827;
  final_pose.orientation.z = 0.0;
  final_pose.orientation.w = 0.0;

  // Process current pose callback
  ros::Rate r(10);
  while (ros::ok() && !has_pose)
  {
    ros::spinOnce();
    r.sleep();
  }
  
  int freq = 100; // 100 Hz
  int ttime = 5; // Trajectory duration in seconds

  line_trajectory(pub, ttime, freq, initial_pose, final_pose, true);

  // Move to edge of arc first
  // geometry_msgs::Pose arc_start;
  // arc_start.position.x = initial_pose.position.x - 0.1;
  // arc_start.position.z = initial_pose.position.y;
  // arc_start.position.z = initial_pose.position.z - 0.1;
  // arc_start.orientation.x = initial_pose.orientation.x;
  // arc_start.orientation.y = initial_pose.orientation.y;
  // arc_start.orientation.z = initial_pose.orientation.z;
  // arc_start.orientation.w = initial_pose.orientation.w;
  // line_trajectory(pub, 5, freq, initial_pose, arc_start);
  // arc_trajectory(pub, ttime, freq, initial_pose, 0.1, true);
  
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
    transform.setOrigin( tf::Vector3(p[0], p[1], p[2]) );
    tf::Quaternion q(pose_a.orientation.x, pose_a.orientation.y, pose_a.orientation.z, pose_a.orientation.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "trajectory"));

    // geometry_msgs::Pose msg;
    // msg.position.x = p[0];
    // msg.position.y = p[1];
    // msg.position.z = p[2];
    // msg.orientation.x = 0.9239;
    // msg.orientation.y = 0.3827;
    // msg.orientation.z = 0.0;
    // msg.orientation.w = 0.0;

    // pub.publish(msg);
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
  // For now let's assume the arc is made on the plane z = a
  // Initial position is considered center

  float t = 0;
  ros::Rate r(frequency);
  while (ros::ok() && t <= duration*frequency)
  {
    geometry_msgs::Pose msg;
    msg.position.x = radius*cos((2*M_PI/(duration*frequency))*t)+center.position.x;
    msg.position.y = radius*sin((2*M_PI/(duration*frequency))*t)+center.position.y;
    msg.position.z = center.position.z;
    msg.orientation.x = 0.9239;
    msg.orientation.y = 0.3827;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;

    pub.publish(msg);
    t += 1;

    ros::spinOnce();
    r.sleep();
  }

  // TODO: Cyclic not working correctly
  if (cyclic) {
    arc_trajectory(pub, duration, frequency, center, cyclic);
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