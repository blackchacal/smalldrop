#include <ros/ros.h>
#include <panda_3dbioprint_vision_system/wound_segmentation.h>

using namespace cv;
using namespace std;
using namespace vision_system;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_node");
  ros::NodeHandle nh;

  std::string filepath = "/home/rtonet/ROS/tese/src/panda_3dbioprint_debug_tools/data/segmentation_points_convexhull.dat";
  WoundSegmentation wseg = WoundSegmentation(800, 800, 0.4, 0.7, -0.15, 0.15);
  wseg.plotWoundConvexHull(filepath);

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}