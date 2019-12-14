#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_pose_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/cartesian_impedance_controller/desired_pose", 10);

  geometry_msgs::Pose msg;
  msg.position.x = 0.5;
  msg.position.y = 0.5;
  msg.position.z = 0.5;
  msg.orientation.x = 0.9239;
  msg.orientation.y = 0.3827;
  msg.orientation.z = 0.0;
  msg.orientation.w = 0.0;

  ros::Rate r(100);
  while (ros::ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
