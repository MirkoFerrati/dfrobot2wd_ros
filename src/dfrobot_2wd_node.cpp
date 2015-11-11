#include <time.h>
#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include "../include/dfrobot_2wd/ros_real_world_serial_communicator.h"

int main ( int argc, char** argv )
{
    //TODO ros param robot name
    std::string robot_name;
    ros::NodeHandle l_node("~");
    if (l_node.getParam("robot_name", robot_name))
    {
      ROS_INFO("Robot name received: %s", robot_name.c_str());
    }
    else
    {
      robot_name="red_blue";
      ROS_INFO("Robot name NOT received: %s", robot_name.c_str());
    }
  
    robot_name = "/"+robot_name+"/cmd_vel";
  
    ros::init(argc,argv,robot_name);
    ros_real_world_serial_communicator comm(robot_name);
    ros::spin();
    return 0;
}
