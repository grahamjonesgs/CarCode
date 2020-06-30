#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_F 0x66
#define KEYCODE_S 0x73
#define KEYCODE_M 0x6D
#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
std_msgs::Bool motor;
std_msgs::Int32 motor_steps;

class TeleopCmd
{
public:
  TeleopCmd();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, _scale_;
  ros::Publisher twist_pub_;
  ros::Publisher motor_pub_;
ros::Publisher motor_steps_pub_;
  
};

TeleopCmd::TeleopCmd():
  linear_(0),
  angular_(0),
  _scale_(0.2)
{
  nh_.param("scale", _scale_, _scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  motor_pub_ = nh_.advertise<std_msgs::Bool>("/motor_on", 1);
  motor_steps_pub_ = nh_.advertise<std_msgs::Int32>("/motor_steps", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_cmd");
  TeleopCmd teleop_cmd;
  

  signal(SIGINT,quit);

  teleop_cmd.keyLoop();
  
  return(0);
}


void TeleopCmd::keyLoop()
{
  char c;
  bool dirty=false;


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
  puts("Use arrow keys to move the car.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_F:
        ROS_DEBUG("FASTER");
        _scale_+=0.1;
        if (_scale_>1.0) _scale_ = 1.0;
        ROS_INFO("Scale is %0.1f",_scale_);
        break;
      case KEYCODE_S:
        ROS_DEBUG("SLOWER");
        _scale_-=0.1;
        if (_scale_<0.0) _scale_ = 0.0;
        ROS_INFO("Scale is %0.1f",_scale_);
        break;
      case KEYCODE_0:
        motor_steps.data=0;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to single steps");
        break;
case KEYCODE_1:
        motor_steps.data=2;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to 2 steps");
        break;
case KEYCODE_2:
        motor_steps.data=4;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to 4 steps");
        break;
case KEYCODE_3:
        motor_steps.data=8;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to 8 steps");
        break;
case KEYCODE_4:
        motor_steps.data=16;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to 16 steps");
        break;
case KEYCODE_5:
        motor_steps.data=32;
        motor_steps_pub_.publish(motor_steps);
        ROS_INFO("Set to 32 steps");
        break;
      case KEYCODE_M:
        ROS_DEBUG("MOTOR");
         
        if (motor.data==0) {
           motor.data=1;
           ROS_INFO("Motor on");
        }
        else {
           motor.data=0;
           ROS_INFO("Motor off");
        }
        motor_pub_.publish(motor);
        
        break;
    }
   

geometry_msgs::Twist twist;
    twist.angular.z = _scale_*angular_;
    twist.linear.x = _scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
  }


  return;
}


