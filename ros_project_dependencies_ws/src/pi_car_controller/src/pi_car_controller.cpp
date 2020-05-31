/*
   Func     Colour  GPIO  pin	pin  GPIO	Colour	Func
   IMU Vcc  Brown   -	    1	  2     -
   IMU SDA          2     3   4     -
   IMU SCL          3     5   6     -
                    4	    7	  8     14
   IMU GND          -	    9	  1     15
                    17    11	12    18  White  RB EN
                    27    13  14	  -   LED GND
                    22	  15	16	  23  LED Red
   ENC VCC          -	    17	18	  24  LED Blue
   ENC L            10	  19	20	  -
   ENC R            9	    21	22	  25
   SW RED           11	  23	24	  8
   ENC GND          -     25	26	  7
                    0	    27	28	  1
   SW YELLOW        5	    29	30	  -
   L CNRL1 Orange	  6	    31	32	  12 Pink   RF EN
   LF EN   Red		  13    33	34	  -
   LB EN   Brown		19    35	36	  16 Brown  R CNRL1
   L CNRL2 Yellow	  26	  37	38	  20 Green  R CTRL2
                    -	    39	40	  21
 */

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>

// Pin definition
#define LEFT_FRONT_EN_PIN 13
#define LEFT_BACK_EN_PIN 19
#define RIGHT_FRONT_EN_PIN 12
#define RIGHT_BACK_EN_PIN 18
#define LEFT_CONTROL_1_PIN 6
#define LEFT_CONTROL_2_PIN 26
#define RIGHT_CONTROL_1_PIN 16
#define RIGHT_CONTROL_2_PIN 20
#define LEFT_SPEED_PIN 10
#define RIGHT_SPEED_PIN 9
#define SWITCH_RED 5
#define SWITCH_YELLOW 11
#define RED_LED_PIN 23
#define BLUE_LED_PIN 24

// Hardware PWM frequency
#define PWM_FREQ 1600
#define MESSAGE_TIMOUT 0.5    // Time to stop if no message received
#define SPEED_TIMOUT 0.5      // Sets frequency of speed check and report
#define LED_FLASH_TIMER 1
#define SPEED_MULTIPLIER 0.0105  // (1/20)*0.21 - divide by number of slots, multipy by curcumference

// Topics
#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_LEFT_SPEED "left_speed"
#define TOPIC_RIGHT_SPEED "right_speed"
#define TOPIC_MOTOR "motor_on"


// Global variables
ros::Timer message_timeout;    // Timer for stop motors if no message
ros::Publisher left_speed_pub;
ros::Publisher right_speed_pub;
int pi;                        // Pi ID from Pigpio
int left_speed_count=0;        // Count of edges on encoder
int right_speed_count=0;       // Count of edges on encoder
bool motor_on=false;           // start in safe mode

void sigintHandler(int sig)
{
  // Shuting down, turn off blue LED and all motors and PWM
        gpio_write(pi,BLUE_LED_PIN,PI_LOW);
        gpio_write(pi,LEFT_CONTROL_1_PIN,PI_LOW);
        gpio_write(pi,LEFT_CONTROL_2_PIN,PI_LOW);
        gpio_write(pi,RIGHT_CONTROL_1_PIN,PI_LOW);
        gpio_write(pi,RIGHT_CONTROL_2_PIN,PI_LOW);
        hardware_PWM(pi,LEFT_FRONT_EN_PIN,PWM_FREQ,0);
        hardware_PWM(pi,LEFT_BACK_EN_PIN,PWM_FREQ,0);
        hardware_PWM(pi,RIGHT_FRONT_EN_PIN,PWM_FREQ,0);
        hardware_PWM(pi,RIGHT_BACK_EN_PIN,PWM_FREQ,0);

        ros::shutdown();
}

float linear_speed(float input_speed){
        // Handle car physics to map required speed to PWM output
        float min_speed=0.3;
        float return_speed;
        if(input_speed==0) {
                return(input_speed);
        }

        return_speed=(input_speed>0) ? (input_speed+min_speed) : (input_speed-min_speed);
        return_speed=return_speed / (1+min_speed); //Simple scaling

        if(return_speed>1) return_speed=1; // Just in case
        if(return_speed<-1) return_speed=-1; // Just in case
        return abs(return_speed);
}

void motor_control(float speed, float turn){
        // Send output to motors based on spped and turn
        // Possitive speed forwards
        // Possitive turn clockwise - slower right
        float left_velocity;
        float right_velocity;

        if(speed<-1||speed>1) return;  // Ignore invalid values
        if(turn<-1||turn>1) return;    // Ignore invalid values
        if(speed==0) {      // For turn on spot
                left_velocity=turn;
                right_velocity=-turn;
        }
        else {
                if (turn < 0) {
                        left_velocity=speed*(1-turn);
                        right_velocity=speed;

                }
                else {
                        left_velocity=speed;
                        right_velocity=speed*(1-turn);
                }
        }
        if(left_velocity!=0||right_velocity!=0) {
                ROS_INFO("left_in=%f, left_map=%f, right_in=%f, right_map=%f",left_velocity,linear_speed(left_velocity),right_velocity,linear_speed(right_velocity));
        }

        hardware_PWM(pi,LEFT_FRONT_EN_PIN,PWM_FREQ,1e6*linear_speed(left_velocity));
        hardware_PWM(pi,LEFT_BACK_EN_PIN,PWM_FREQ,1e6*linear_speed(left_velocity));
        hardware_PWM(pi,RIGHT_FRONT_EN_PIN,PWM_FREQ,1e6*linear_speed(right_velocity));
        hardware_PWM(pi,RIGHT_BACK_EN_PIN,PWM_FREQ,1e6*linear_speed(right_velocity));

        if ((speed==0 && turn==0)||motor_on==false) {
                gpio_write(pi,LEFT_CONTROL_1_PIN,PI_LOW);
                gpio_write(pi,LEFT_CONTROL_2_PIN,PI_LOW);
                gpio_write(pi,RIGHT_CONTROL_1_PIN,PI_LOW);
                gpio_write(pi,RIGHT_CONTROL_2_PIN,PI_LOW);
        }
        else {
                (left_velocity < 0) ? gpio_write(pi,LEFT_CONTROL_1_PIN,PI_HIGH) : gpio_write(pi,LEFT_CONTROL_1_PIN,PI_LOW);
                (right_velocity < 0) ? gpio_write(pi,RIGHT_CONTROL_1_PIN,PI_HIGH) : gpio_write(pi,RIGHT_CONTROL_1_PIN,PI_LOW);
                (left_velocity < 0) ? gpio_write(pi,LEFT_CONTROL_2_PIN,PI_LOW) : gpio_write(pi,LEFT_CONTROL_2_PIN,PI_HIGH);
                (right_velocity < 0) ? gpio_write(pi,RIGHT_CONTROL_2_PIN,PI_LOW) : gpio_write(pi,RIGHT_CONTROL_2_PIN,PI_HIGH);
        }
}

void message_timeout_callback (const ros::TimerEvent&){
        // ROS callback if not message received to stop motors
        motor_control(0.0,0.0);
}

void LED_timer_callback (const ros::TimerEvent&){
        // ROS callback for LED messages
        if (!motor_on) {
                gpio_write(pi,BLUE_LED_PIN,PI_HIGH);
        }
        else {
                gpio_write(pi,BLUE_LED_PIN,!gpio_read(pi,BLUE_LED_PIN));
        }

}

static void speed_callback(int pi,uint32_t gpio, uint32_t level, uint32_t tick){
        // Pigpio callback on speed encoded pin rising edge
        gpio==LEFT_SPEED_PIN ? left_speed_count++ : right_speed_count++;
}


void speed_timer_callback (const ros::TimerEvent&){
        // ROS callback on time to send wheel speeds
        std_msgs::Float32 left_speed;
        std_msgs::Float32 right_speed;
        left_speed.data = left_speed_count * SPEED_MULTIPLIER / SPEED_TIMOUT;
        right_speed.data =  right_speed_count * SPEED_MULTIPLIER / SPEED_TIMOUT;

        left_speed_pub.publish(left_speed);
        right_speed_pub.publish(right_speed);
        left_speed_count=0;
        right_speed_count=0;
}

void velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
        // ROS callback on message received
        message_timeout.setPeriod(ros::Duration(MESSAGE_TIMOUT),true);  // reset no message timeout
        motor_control(msg->linear.x,msg->angular.z);
}

void motor_callback(const std_msgs::Bool::ConstPtr& msg)
{
        // ROS callback on message received
        motor_on=msg->data;
}

int main (int argc, char **argv)
{
        ros::init(argc, argv, "pi_car_controller", ros::init_options::NoSigintHandler);
        ROS_INFO("Started Pi Car Controller");
        ros::NodeHandle nh;
        signal(SIGINT, sigintHandler);
        signal(SIGTERM, sigintHandler);
        signal(SIGKILL, sigintHandler);
        if (pi=pigpio_start(NULL,NULL)<0) {
                ROS_INFO("gpio inti failed");
                return 1;
        }

        set_mode(pi,LEFT_FRONT_EN_PIN,  PI_OUTPUT);
        set_mode(pi,LEFT_BACK_EN_PIN,  PI_OUTPUT);
        set_mode(pi,LEFT_CONTROL_1_PIN,  PI_OUTPUT);
        set_mode(pi,LEFT_CONTROL_2_PIN,  PI_OUTPUT);
        set_mode(pi,RIGHT_FRONT_EN_PIN,  PI_OUTPUT);
        set_mode(pi,RIGHT_BACK_EN_PIN,  PI_OUTPUT);
        set_mode(pi,RIGHT_CONTROL_1_PIN,  PI_OUTPUT);
        set_mode(pi,RIGHT_CONTROL_2_PIN,  PI_OUTPUT);
        set_mode(pi,RED_LED_PIN,  PI_OUTPUT);
        set_mode(pi,BLUE_LED_PIN,  PI_OUTPUT);
        set_mode(pi,LEFT_SPEED_PIN,  PI_INPUT);
        set_mode(pi,RIGHT_SPEED_PIN,  PI_INPUT);

        gpio_write(pi,BLUE_LED_PIN,PI_HIGH);

        // Set up ROS
        ros::Subscriber sub_velocity = nh.subscribe(TOPIC_CMD_VEL,10,velocity_callback);
        ros::Subscriber sub_motor = nh.subscribe(TOPIC_MOTOR,10,motor_callback);
        message_timeout = nh.createTimer(ros::Duration(MESSAGE_TIMOUT), message_timeout_callback);
        ros::Timer speed_timer = nh.createTimer(ros::Duration(SPEED_TIMOUT), speed_timer_callback);
        ros::Timer LED_timer = nh.createTimer(ros::Duration(LED_FLASH_TIMER), LED_timer_callback);
        left_speed_pub = nh.advertise<std_msgs::Float32>(TOPIC_LEFT_SPEED, 2);
        right_speed_pub = nh.advertise<std_msgs::Float32>(TOPIC_RIGHT_SPEED, 2);


        callback(pi,LEFT_SPEED_PIN,RISING_EDGE,speed_callback);
        callback(pi,RIGHT_SPEED_PIN,RISING_EDGE,speed_callback);
        ros::spin();
}
