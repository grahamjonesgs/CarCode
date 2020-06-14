/*

 */
#include <string>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>
#include "wili9340.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/LaserScan.h"

// Global variables
ros::Timer imu_timer;    // Timer for stop motors if no message
ros::Publisher imu_pub;

FontxFile fx32G[2];
FontxFile fx24G[2];
FontxFile fx16G[2];
FontxFile fx32M[2];
FontxFile fx24M[2];
FontxFile fx16M[2];

#define TOPIC_CMD_VEL "cmd_vel"
#define TOPIC_LEFT_SPEED "left_speed"
#define TOPIC_RIGHT_SPEED "right_speed"
#define TOPIC_MOTOR "motor_on"
#define TOPIC_IMU_RAW "imu/data_raw"
#define TOPIC_IMU "imu/data"
#define TOPIC_SCAN "scan"
#define TOPIC_MAG "imu/mag"
#define TOPIC_MSG "msg"

#define STD_HEIGHT 20
#define TOP_TITTLE 10
#define TOP_LEFT_SPEED 30
#define TOP_RIGHT_SPEED 50
#define TOP_IMU_RAW 70
#define TOP_IMU 90
#define TOP_MAG 110
#define TOP_SCAN 130
#define TOP_CMD 150
#define TOP_MOTOR 170
#define TOP_MSG 190
#define STATIC_LEFT 5
#define STATIC_RIGHT 120
#define VARIABLE_LEFT 130
#define VARIABLE_RIGHT 310

struct  LcdWindow* mainWindow;
struct LcdWindow* wsLeftSpeed;
struct LcdWindow* wsRightSpeed;
struct LcdWindow* wsIMURaw;
struct LcdWindow* wsIMU;
struct LcdWindow* wsMag;
struct LcdWindow* wsScan;
struct LcdWindow* wsCMD;
struct LcdWindow* wsMotor;
struct LcdWindow* wsMsg;

struct LcdWindow* wvLeftSpeed;
struct LcdWindow* wvRightSpeed;
struct LcdWindow* wvIMURaw;
struct LcdWindow* wvIMU;
struct LcdWindow* wvMag;
struct LcdWindow* wvScan;
struct LcdWindow* wvCMD;
struct LcdWindow* wvMotor;
struct LcdWindow* wvMsg;

void load_fonts(){
        Fontx_init(fx32G,"./fontx/ILGH32XB.FNT","./fontx/ILGZ32XB.FNT"); // 16x32Dot Gothic
        Fontx_init(fx24G,"./fontx/ILGH24XB.FNT","./fontx/ILGZ24XB.FNT"); // 12x24Dot Gothic
        Fontx_init(fx16G,"./fontx/ILGH16XB.FNT","./fontx/ILGZ16XB.FNT"); // 8x16Dot Gothic
        Fontx_init(fx32M,"./fontx/ILMH32XF.FNT","./fontx/ILMZ32XF.FNT"); // 16x32Dot Mincyo
        Fontx_init(fx24M,"./fontx/ILMH24XF.FNT","./fontx/ILMZ24XF.FNT"); // 12x24Dot Mincyo
        Fontx_init(fx16M,"./fontx/ILMH16XB.FNT","./fontx/ILMZ16XF.FNT"); // 8x16Dot Mincyo
}

void speed_timer_callback (const ros::TimerEvent&)
{
        // ROS callback
}

void cmd_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"lin %3.2f, ang %3.2f",msg->linear.x,msg->angular.z);
        lcdDrawUTF8String(wvCMD,fx16G, 0,0, message, WHITE);
}

void motor_callback(const std_msgs::Bool::ConstPtr& msg)
{
        // ROS callback on message received
        lcdFillScreen( wvMotor, BLACK);
        if(msg->data) {
                lcdDrawUTF8String(wvMotor,fx16G, 0,0, "On", WHITE);
        }
        else {
                lcdDrawUTF8String(wvMotor,fx16G, 0,0, "Off", WHITE);
        }
}

void left_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%6.2f",msg->data);
        lcdDrawUTF8String(wvLeftSpeed,fx16G, 0,0, message, WHITE);
}

void right_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%6.2f",msg->data);
        lcdDrawUTF8String(wvRightSpeed,fx16G, 0,0, message, WHITE);
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%+6.1f %+6.1f %+6.1f",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
        lcdDrawUTF8String(wvIMU,fx16G, 0,0, message, WHITE);
}

void imu_raw_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%+6.1f %+6.1f %+6.1f",msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
        lcdDrawUTF8String(wvIMURaw,fx16G, 0,0, message, WHITE);
}

void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%+6.0f %+6.0f %+6.0f",msg->magnetic_field.x,msg->magnetic_field.y,msg->magnetic_field.z);
        lcdDrawUTF8String(wvMag,fx16G, 0,0, message, WHITE);
}

void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        float min=999.0;
        int pos=0;

        for (int i=0; i<msg->ranges.size(); i++) {
                if (msg->ranges[i]<min) {
                        min=msg->ranges[i];
                        pos=i;
                }
        }
        snprintf(message,sizeof(message),"%3iDeg %7.3fcm",pos,msg->ranges[pos]);
        lcdDrawUTF8String(wvScan,fx16G, 0,0, message, WHITE);
}

void msg_callback(const std_msgs::String::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];
        snprintf(message,sizeof(message),"%s                                           ",msg->data.c_str());
        lcdDrawUTF8String(wvMsg,fx16G, 0,0, message, WHITE);
}

void SigintHandler(int sig)
{
        ros::shutdown();
        lcdFillScreen( mainWindow, BLACK);
}
void setup_screen()
{
        // Set-up basic screen info
        int screenWidth = 320;
        int screenHeight = 240;
        int offsetx = 0;
        int offsety = 0;
        int colour;

        load_fonts();
        mainWindow = lcdInit(offsetx, offsety, screenWidth, screenHeight, false);
        lcdReset();
        lcdSetup();
        lcdFillScreen( mainWindow, BLACK);
        lcdSetFontFill(BLACK);
        colour = WHITE;
        lcdSetFontDirection(DIRECTION0);

        wsLeftSpeed=lcdWindowInit(STATIC_LEFT,TOP_LEFT_SPEED,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsRightSpeed=lcdWindowInit(STATIC_LEFT,TOP_RIGHT_SPEED,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsIMURaw=lcdWindowInit(STATIC_LEFT,TOP_IMU_RAW,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsIMU=lcdWindowInit(STATIC_LEFT,TOP_IMU,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsMag=lcdWindowInit(STATIC_LEFT,TOP_MAG,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsScan=lcdWindowInit(STATIC_LEFT,TOP_SCAN,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsCMD=lcdWindowInit(STATIC_LEFT,TOP_CMD,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsMotor=lcdWindowInit(STATIC_LEFT,TOP_MOTOR,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsMsg=lcdWindowInit(STATIC_LEFT,TOP_MSG,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);

        wvLeftSpeed=lcdWindowInit(VARIABLE_LEFT,TOP_LEFT_SPEED,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvRightSpeed=lcdWindowInit(VARIABLE_LEFT,TOP_RIGHT_SPEED,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvIMURaw=lcdWindowInit(VARIABLE_LEFT,TOP_IMU_RAW,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvIMU=lcdWindowInit(VARIABLE_LEFT,TOP_IMU,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvMag=lcdWindowInit(VARIABLE_LEFT,TOP_MAG,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvScan=lcdWindowInit(VARIABLE_LEFT,TOP_SCAN,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvCMD=lcdWindowInit(VARIABLE_LEFT,TOP_CMD,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvMotor=lcdWindowInit(VARIABLE_LEFT,TOP_MOTOR,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvMsg=lcdWindowInit(VARIABLE_LEFT,TOP_MSG,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);

        lcdDrawUTF8String(wsLeftSpeed,fx16G, 0,0, "Left speed", WHITE);
        lcdDrawUTF8String(wsRightSpeed,fx16G, 0,0, "Right speed", WHITE);
        lcdDrawUTF8String(wsIMURaw,fx16G, 0,0, "IMU Raw", WHITE);
        lcdDrawUTF8String(wsIMU,fx16G, 0,0, "IMU", WHITE);
        lcdDrawUTF8String(wsMag,fx16G, 0,0, "Magnetic", WHITE);
        lcdDrawUTF8String(wsScan,fx16G, 0,0, "Laser scan", WHITE);
        lcdDrawUTF8String(wsCMD,fx16G, 0,0, "Command", WHITE);
        lcdDrawUTF8String(wsMotor,fx16G, 0,0, "Motor", WHITE);
        lcdDrawUTF8String(wsMsg,fx16G, 0,0, "Message", WHITE);
}

int main (int argc, char **argv)
{
        ros::init(argc, argv, "pi_car_lcd", ros::init_options::NoSigintHandler);
        ROS_INFO("Started Pi Car LCD");
        signal(SIGINT, SigintHandler);
        ros::NodeHandle nh;

        //imu_timer = nh.createTimer(ros::Duration(IMU_TIMER), imu_timer_callback);
        setup_screen();

        ros::Subscriber sub_cmd = nh.subscribe(TOPIC_CMD_VEL,1,cmd_callback);
        ros::Subscriber sub_motor = nh.subscribe(TOPIC_MOTOR,1,motor_callback);
        ros::Subscriber sub_left_speed = nh.subscribe(TOPIC_LEFT_SPEED,1,left_speed_callback);
        ros::Subscriber sub_right_speed = nh.subscribe(TOPIC_RIGHT_SPEED,1,right_speed_callback);
        ros::Subscriber sub_imu = nh.subscribe(TOPIC_IMU,1,imu_callback);
        ros::Subscriber sub_imu_raw = nh.subscribe(TOPIC_IMU_RAW,1,imu_raw_callback);
        ros::Subscriber sub_mag = nh.subscribe(TOPIC_MAG,1,mag_callback);
        ros::Subscriber sub_scan = nh.subscribe(TOPIC_SCAN,1,scan_callback);
        ros::Subscriber sub_msg = nh.subscribe(TOPIC_MSG,1,msg_callback);

        ros::spin();
}
