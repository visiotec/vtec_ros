#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <std_msgs/Char.h>
#include <ros/ros.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

ros::Publisher cmd_pub;
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void keyLoop()
{
  char c;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");

  std_msgs::Char char_msg;

  for(;;)
  {
    // get the next event from the keyboard  
    if(::read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
  
    char_msg.data = c;
    cmd_pub.publish(char_msg);    
  }


  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");

  ros::NodeHandle nh;

  cmd_pub = nh.advertise<std_msgs::Char>("track_cmd", 1);

  signal(SIGINT,quit);

  keyLoop();
  
  return(0);
}

