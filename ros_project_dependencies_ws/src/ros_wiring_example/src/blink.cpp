#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <iostream>
#include <wiringPi.h>
#define LED_PIN 0 // change pin number here

void blink_callback(const std_msgs::Bool::ConstPtr& msg)
{
  if(msg->data == 1) {
    digitalWrite(LED_PIN, HIGH);
    ROS_INFO("LED ON");
  }
  if(msg->data == 0) {
    digitalWrite(LED_PIN, LOW);
    ROS_INFO("LED OFF");
  }
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "blink_led");
    ROS_INFO("Started Blink Node");
    ros::NodeHandle nh;
    wiringPiSetup();

    pinMode(LED_PIN, OUTPUT);
    ROS_INFO("GPIO has been set as OUTPUT.");

    ros::Subscriber sub = nh.subscribe("led_blink",10,blink_callback);
    ros::spin();

}
