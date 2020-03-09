// Copyright (c) 2019-2020 Ricardo Tonet
// Use of this source code is governed by the MIT license, see LICENSE

/**
 * \file gazebo_models_to_rviz_node.cpp 
 * \brief Node to handle publishing of gazebo models to rviz as markers.
 */

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

std::string gazebo_modelstates_topic = "/gazebo/model_states";
std::string rviz_markers_topic = "/smalldrop_rviz/gazebo_rviz_markers";

ros::Publisher pub;

/**
 * \fn void gazeboModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg)
 * \brief Callback function that publishes the gazebo models on rviz.
 * 
 * \param msg Gazebo message with the models state.
 */
void gazeboModelStatesCallback(const gazebo_msgs::ModelStatesConstPtr msg)
{
  static tf::TransformBroadcaster br;

  unsigned int size = msg->name.size();
  unsigned int id = 0;

  for (size_t i = 0; i < size; i++)
  {
    if (msg->name[i].compare("ground_plane") != 0 && 
      msg->name[i].compare("patient_room") != 0 && 
      msg->name[i].compare("panda") != 0 &&
      msg->name[i].compare("tv") != 0)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "smalldrop_robot_arm";
      marker.id = id;
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = msg->pose[i];

      // Adjust color and scale depending on the model
      if (msg->name[i].compare("emergency_console") == 0)
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
      }
      else if (msg->name[i].compare("hospital_bed") == 0)
      {
        marker.scale.x = 1.1;
        marker.scale.y = 1.3;
        marker.scale.z = 2.1;
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
      }
      else if (msg->name[i].compare("serum_support") == 0)
      {
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 0;
      }
      else if (msg->name[i].compare("service_cart") == 0)
      {
          marker.scale.x = 0.01;
          marker.scale.y = 0.01;
          marker.scale.z = 0.01;
          marker.color.r = 0.8;
          marker.color.g = 0.8;
          marker.color.b = 0.8;
      }
      else if (msg->name[i].compare("support_table") == 0)
      {
          marker.scale.x = 0.005;
          marker.scale.y = 0.005;
          marker.scale.z = 0.01;
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
      } else
      {
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.r = 0.8;
        marker.color.g = 0.8;
        marker.color.b = 0.8;
      }
      
      marker.color.a = 1.0;
      marker.mesh_resource = "package://smalldrop_robot_arm/models/"+msg->name[i]+"/meshes/"+msg->name[i]+".dae";
      marker.mesh_use_embedded_materials = true;
      pub.publish( marker );
      id++;

      // Broadcast tf of models pose axis
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(msg->pose[i].position.x, msg->pose[i].position.y, msg->pose[i].position.z));
      tf::Quaternion q(msg->pose[i].orientation.x, msg->pose[i].orientation.y, msg->pose[i].orientation.z, msg->pose[i].orientation.w);
      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", msg->name[i]));
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gazebo_models_to_rviz");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe(gazebo_modelstates_topic, 10, gazeboModelStatesCallback);
  pub = nh.advertise<visualization_msgs::Marker>(rviz_markers_topic, 10);

  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
}
