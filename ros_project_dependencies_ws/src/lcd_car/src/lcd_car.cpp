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
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

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

#define STD_HEIGHT 20
#define TOP_TITTLE 10
#define TOP_LEFT_SPEED 30
#define TOP_RIGHT_SPEED 50
#define TOP_IMU_RAW 70
#define TOP_IMU 90
#define TOP_MAG 110
#define TOP_SCAN 130
#define TOP_CMD 150
#define STATIC_LEFT 5
#define STATIC_RIGHT 120
#define VARIABLE_LEFT 130
#define VARIABLE_RIGHT 230

struct  LcdWindow* mainWindow;

struct LcdWindow* wsLeftSpeed;
struct LcdWindow* wsRightSpeed;
struct LcdWindow* wsIMURaw;
struct LcdWindow* wsIMU;
struct LcdWindow* wsMag;
struct LcdWindow* wsScan;
struct LcdWindow* wsCMD;

struct LcdWindow* wvLeftSpeed;
struct LcdWindow* wvRightSpeed;
struct LcdWindow* wvIMURaw;
struct LcdWindow* wvIMU;
struct LcdWindow* wvMag;
struct LcdWindow* wvScan;
struct LcdWindow* wvCMD;



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

void velocity_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
        // ROS callback on message received
        char message[256];

        snprintf(message,sizeof(message),"Speed %0.2f, angle %0.2f",msg->linear.x,msg->angular.z);
        lcdDrawUTF8String(wvCMD,fx16G, 0,0, message, WHITE);
        //message_timeout.setPeriod(ros::Duration(MESSAGE_TIMOUT),true);  // reset no message timeout
        //motor_control(msg->linear.x,msg->angular.z);
}

void motor_callback(const std_msgs::Bool::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
}

void left_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
}

void right_speed_callback(const std_msgs::Float32::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
}

void imu_raw_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
}

void mag_callback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
        // ROS callback on message received
        //motor_on=msg->data;
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
        colour = WHITE;

        lcdSetFontDirection(DIRECTION0);

        wsLeftSpeed=lcdWindowInit(STATIC_LEFT,TOP_LEFT_SPEED,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsRightSpeed=lcdWindowInit(STATIC_LEFT,TOP_RIGHT_SPEED,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsIMURaw=lcdWindowInit(STATIC_LEFT,TOP_IMU_RAW,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsIMU=lcdWindowInit(STATIC_LEFT,TOP_IMU,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsMag=lcdWindowInit(STATIC_LEFT,TOP_MAG,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsScan=lcdWindowInit(STATIC_LEFT,TOP_SCAN,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);
        wsCMD=lcdWindowInit(STATIC_LEFT,TOP_CMD,STATIC_RIGHT-STATIC_LEFT,STD_HEIGHT);

        wvLeftSpeed=lcdWindowInit(VARIABLE_LEFT,TOP_LEFT_SPEED,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvRightSpeed=lcdWindowInit(VARIABLE_LEFT,TOP_RIGHT_SPEED,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvIMURaw=lcdWindowInit(VARIABLE_LEFT,TOP_IMU_RAW,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvIMU=lcdWindowInit(VARIABLE_LEFT,TOP_IMU,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvMag=lcdWindowInit(VARIABLE_LEFT,TOP_MAG,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvScan=lcdWindowInit(VARIABLE_LEFT,TOP_SCAN,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);
        wvCMD=lcdWindowInit(VARIABLE_LEFT,TOP_CMD,VARIABLE_RIGHT-VARIABLE_LEFT,STD_HEIGHT);

        lcdDrawUTF8String(wsLeftSpeed,fx16G, 0,0, "Left speed", WHITE);
        lcdDrawUTF8String(wsRightSpeed,fx16G, 0,0, "Right speed", WHITE);
        lcdDrawUTF8String(wsIMURaw,fx16G, 0,0, "IMU Raw", WHITE);
        lcdDrawUTF8String(wsIMU,fx16G, 0,0, "IMU", WHITE);
        lcdDrawUTF8String(wsMag,fx16G, 0,0, "Magnetic", WHITE);
        lcdDrawUTF8String(wsScan,fx16G, 0,0, "Laser scan", WHITE);
        lcdDrawUTF8String(wsCMD,fx16G, 0,0, "Command", WHITE);


}

int main (int argc, char **argv)
{

        ros::init(argc, argv, "pi_car_lcd", ros::init_options::NoSigintHandler);
        ROS_INFO("Started Pi Car LCD");
        ros::NodeHandle nh;

        // Set up ROS
        //imu_timer = nh.createTimer(ros::Duration(IMU_TIMER), imu_timer_callback);

        uint16_t colour;
        unsigned char utf8[64];
        setup_screen();


        /*strncpy((char *)utf8, "10,10", sizeof(utf8));
           lcdDrawUTF8String(mainWindow,fx16G, 10,10, utf8, colour);

           strncpy((char *)utf8, "10,200", sizeof(utf8));
           lcdDrawUTF8String(mainWindow,fx16G, 10,200, utf8, colour);

           strncpy((char *)utf8, "100,100", sizeof(utf8));
           lcdDrawUTF8String(mainWindow,fx16G, 100,100, utf8, colour);

           strncpy((char *)utf8, "100,200", sizeof(utf8));
           lcdDrawUTF8String(mainWindow,fx16G, 100,200, utf8, colour);

           lcdDrawPixel(mainWindow,310,230,RED);

           struct LcdWindow* subWindow = lcdWindowInit(100,100,95,50);
           lcdDrawLine(subWindow,1,1,200,200,BLUE);
           strncpy((char *)utf8, "subsuub123456789012345678901234567890", sizeof(utf8));
           lcdDrawUTF8String(subWindow,fx16G, 10,10, utf8, GREEN);
           //lcdDrawFillRect(mainWindow,0,0,50,51,CYAN);
           lcdDrawFillRect(mainWindow,0,0,50,51,GREEN);*/

        ros::Subscriber sub_velocity = nh.subscribe(TOPIC_CMD_VEL,1,velocity_callback);
        ros::Subscriber sub_motor = nh.subscribe(TOPIC_MOTOR,1,motor_callback);
        ros::Subscriber sub_left_speed = nh.subscribe(TOPIC_LEFT_SPEED,1,left_speed_callback);
        ros::Subscriber sub_right_speed = nh.subscribe(TOPIC_RIGHT_SPEED,1,right_speed_callback);
        ros::Subscriber sub_imu = nh.subscribe(TOPIC_IMU,1,imu_callback);
        ros::Subscriber sub_imu_raw = nh.subscribe(TOPIC_IMU_RAW,1,imu_raw_callback);
        ros::Subscriber sub_mag = nh.subscribe(TOPIC_MAG,1,mag_callback);



        ros::spin();
}
