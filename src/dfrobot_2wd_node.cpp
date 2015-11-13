#include <time.h>
#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include "../include/dfrobot_2wd/ros_real_world_serial_communicator.h"

int main ( int argc, char** argv )
{
    ros::init(argc, argv, "dfrobot_2wd");
  
    std::string l_robot_name;
    ros::NodeHandle l_node("~");
    if (l_node.getParam("robot_name", l_robot_name))
    {
      ROS_INFO("Robot name received: %s", l_robot_name.c_str());
    }
    else
    {
      l_robot_name="red_blue";
      ROS_INFO("Robot name NOT received: %s", l_robot_name.c_str());
    }
    
    std::string l_port_name;
    if (l_node.getParam("port_name", l_port_name))
    {
      ROS_INFO("Port name received: %s", l_port_name.c_str());
    }
    else
    {
      l_port_name="/dev/ttyACM0";
      ROS_INFO("Port name NOT received: %s", l_port_name.c_str());
    }
    l_robot_name = "/"+l_robot_name+"/cmd_vel";
    
    ROS_INFO("Subscribe to: %s", l_robot_name.c_str());
    
    ros_real_world_serial_communicator comm(l_robot_name, l_port_name);
    ros::spin();
    return 0;
}
