#include <time.h>
#include <iostream>
#include <string>
#include <thread>
#include <ros/ros.h>
#include "../include/dfrobot_2wd/ros_real_world_serial_communicator.h"

int main ( int argc, char** argv )
{
    //TODO ros param robot name
    std::string robot_name="pippo";
    ros::init(argc,argv,robot_name);
    ros_real_world_serial_communicator comm(robot_name);
    ros::spin();
    return 0;
}
