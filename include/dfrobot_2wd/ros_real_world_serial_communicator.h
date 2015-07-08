#ifndef ROS_REAL_WORLD_SERIAL_COMMUNICATOR_H
#define ROS_REAL_WORLD_SERIAL_COMMUNICATOR_H
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <string>
#include <ostream>
#include <stdio.h>
#include <fcntl.h>
#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#define ARDUINO_COMMAND_CODE 7

class ros_real_world_serial_communicator
{
public:
    ros_real_world_serial_communicator(std::string agent_name);
    void send_control_command( double v, double w);

    ~ros_real_world_serial_communicator();

private:
geometry_msgs::Twist packet_received;
int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);

int fd; //File descriptor

void cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd);

ros::NodeHandle nh;
ros::Subscriber sub;

};


#endif //ROS_REAL_WORLD_SERIAL_COMMUNICATOR_H
