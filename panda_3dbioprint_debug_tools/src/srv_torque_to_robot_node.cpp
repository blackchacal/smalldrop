#include <ros/ros.h>
#include <std_srvs/SetBool.h>

bool send_torque = true;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "send_torque_to_robot");
  ros::NodeHandle nh;

  ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("/cartesian_impedance_controller/send_torques_to_robot");

  // Process command-line arguments
  int opt;
  const char* const short_opts = ":hT:";
  while ((opt = getopt(argc, argv, short_opts)) != -1)
  {
    switch (opt)
    {
      case 'T':
        send_torque = static_cast<bool>(std::stoi(optarg));
        break;
      case 'h':
      default:
        std::cout << "Help:" << std::endl; 
        std::cout << "send_torque2robot -h -T <0/1>" << std::endl; 
        std::cout << "Default value is 1." << std::endl; 
        return 0;
    }
  } 

  std_srvs::SetBool srv;
  srv.request.data = send_torque;

  if (client.call(srv))
  {
    ROS_INFO("Response: %d", srv.response.success);
  }
  else
  {
    ROS_ERROR("Failed to call service 'send_torques_to_robot'.");
    return 1;
  }
  
  return 0;
}
