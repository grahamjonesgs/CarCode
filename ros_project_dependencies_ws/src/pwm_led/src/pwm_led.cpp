#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <pigpiod_if2.h>
#define LED_PIN 13 // change pin number here
int pi;
void blink_callback(const std_msgs::Bool::ConstPtr& msg)
{
        if(msg->data == 1) {
                //set_PWM_dutycycle(pi,LED_PIN, 250);
                hardware_PWM(pi,LED_PIN,800,1e6);
                ROS_INFO("LED ON");
        }
        if(msg->data == 0) {
                //set_PWM_dutycycle(pi,LED_PIN, 200);
                hardware_PWM(pi,LED_PIN,800,1e6*0.25);
                ROS_INFO("LED OFF");
        }
}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  float speed;
  float turn;
        speed=msg->linear.x;
        turn=msg->angular.z;
        hardware_PWM(pi,LED_PIN,800,1e6*speed);
        ROS_INFO("Float control");
        /*if(msg->data == 1) {
                //set_PWM_dutycycle(pi,LED_PIN, 250);
                hardware_PWM(pi,LED_PIN,800,1e6);
                ROS_INFO("LED ON");
        }
        if(msg->data == 0) {
                //set_PWM_dutycycle(pi,LED_PIN, 200);
                hardware_PWM(pi,LED_PIN,800,1e6*0.25);
                ROS_INFO("LED OFF");
        }*/
}

int main (int argc, char **argv)
{

        ros::init(argc, argv, "pwm_led");
        ROS_INFO("Started pwm LED Node");
        ros::NodeHandle nh;
        if (pi=pigpio_start(NULL,NULL)<0) {
                ROS_INFO("gpio inti failed");
                return 1;
        }
        else {
                ROS_INFO("gpio init ok");
        }

        set_mode(pi,LED_PIN,  PI_OUTPUT);
        //hardware_PWM(pi,LED_PIN, )

        ROS_INFO("GPIO has been set as OUTPUT.");

        ros::Subscriber sub_flash = nh.subscribe("led_pwm",10,blink_callback);
ros::Subscriber sub_velocity = nh.subscribe("velocity",10,velocity_callback);
        ros::spin();

}
