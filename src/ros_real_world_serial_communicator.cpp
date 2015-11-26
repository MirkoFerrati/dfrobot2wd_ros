#include "../include/dfrobot_2wd/ros_real_world_serial_communicator.h"
#include <string>
#include <ros/console.h>
using namespace std;

// Record the execution time of some code, in milliseconds.
int64_t get_tick_count()
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return (int64_t)tp.tv_sec*1000000000 + tp.tv_nsec;
}

#define DECLARE_TIMING(s)  int64_t timeStart_##s; double timeDiff_##s; double timeTally_##s = 0; int countTally_##s = 0
#define START_TIMING(s)    timeStart_##s = get_tick_count()
#define STOP_TIMING(s) 	   timeDiff_##s = (double)(get_tick_count() - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++
#define GET_TIMING(s) 	   (double)(timeDiff_##s / ( 1e9*1000.0))
#define GET_AVERAGE_TIMING(s)   (double)(countTally_##s ? timeTally_##s/ ((double)countTally_##s *  1e9*1000.0) : 0)
#define CLEAR_AVERAGE_TIMING(s) timeTally_##s = 0; countTally_##s = 0


ros_real_world_serial_communicator::ros_real_world_serial_communicator(std::string agent_name, std::string portname)
{
    ROS_INFO("Subscribe to: %s", agent_name.c_str());
    ROS_INFO("Publish on: %s", portname.c_str());
  
    sub = nh.subscribe<geometry_msgs::Twist>(agent_name,1,&ros_real_world_serial_communicator::cmd_callback,this);
    fd = open (portname.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        ROS_ERROR("error %d opening %s: %s", errno, portname.c_str(), strerror (errno));
        return;
    }

    set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 0);                // set no blocking

    write (fd, "1;2;", 4);           // send 7 character greeting
    char buf [100];
    int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
    std::cout<<buf<<endl;
    sleep(1);
}

void ros_real_world_serial_communicator::cmd_callback(const geometry_msgs::Twist::ConstPtr& cmd)
{
  ROS_INFO("Linear %.4f, Angular %.4f\n", cmd->linear.x, cmd->angular.z);
  send_control_command(cmd->linear.x,cmd->angular.z);
}

void ros_real_world_serial_communicator::set_blocking(int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        ROS_ERROR("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        ROS_ERROR("error %d setting term attributes", errno);
    
    ROS_INFO("Blocking is activated!\n");
}


int ros_real_world_serial_communicator::set_interface_attribs(int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        ROS_ERROR("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // ignore break signal
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        ROS_ERROR("error %d from tcsetattr", errno);
        return -1;
    }
    
    ROS_INFO("Interface Attributes are set!\n");
    
    return 0;

}

void ros_real_world_serial_communicator::send_control_command(double v, double w)
{
    DECLARE_TIMING(myTimer);
    START_TIMING(myTimer);
    char buf[100];
    sprintf(buf,"%i,%f,%f;",ARDUINO_COMMAND_CODE,v,w);
    //tcflush(fd,TCIOFLUSH);
    write(fd,buf,strlen(buf));
    //tcdrain(fd);
    ROS_DEBUG("SerialMessage Sent: %i, %f, %f \n",ARDUINO_COMMAND_CODE,v,w);
    STOP_TIMING(myTimer);
    ROS_DEBUG("Execution time: %f ms.\n", GET_TIMING(myTimer) );
    ROS_DEBUG("Average time: %f ms per iteration.\n", GET_AVERAGE_TIMING(myTimer) );

}


ros_real_world_serial_communicator::~ros_real_world_serial_communicator()
{

}
