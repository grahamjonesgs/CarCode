/*
   Func     Colour  GPIO  pin	pin  GPIO	Colour	Func
   MAG Vcc  Brown   -	    1	  2     -
   MAG SDA          2     3   4     -
   MAG SCL          3     5   6     -
                    4	    7	  8     14
   MAG GND          -	    9	  1     15
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
   Normalisation values
   Ave G -0.0617,0.0115,0.0011  A -0.6567,-1.78,10.47
 */

#include <ros/ros.h>
#include "sensor_msgs/MagneticField.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <stdlib.h>
#include <signal.h>

// Topics
#define TOPIC_MAG "imu/mag"

// Timers
#define MAG_TIMER 0.02    // Interval betten MAG readings

// MPU6050 definitions
/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D
//#define QMC5883L_ADDR 0x01

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif



// Global variables
ros::Timer mag_timer;
ros::Publisher mag_pub;
int pi;                        // Pi ID from Pigpio
int i2ch;                      // I2C handle

int16_t xhigh, xlow;
int16_t yhigh, ylow;
uint8_t mode;
uint8_t rate;
uint8_t range;
uint8_t oversampling;

// Normalisation values
float DigitPermg;

short read_raw_data(int addr){
        short high_byte,low_byte,value;
        high_byte = i2c_read_byte_data(pi,i2ch, addr+1);
        low_byte = i2c_read_byte_data(pi,i2ch, addr);
        value = (high_byte << 8) | low_byte;
        return value;
}

void reconfig()
{
        i2c_write_byte_data (pi,i2ch,QMC5883L_CONFIG,oversampling|range|rate|mode);
}

void reset()
{
        i2c_write_byte_data (pi,i2ch,QMC5883L_RESET,0x01);
        reconfig();
}

void setOversampling( int x )
{
        switch(x) {
        case 512:
                oversampling = QMC5883L_CONFIG_OS512;
                break;
        case 256:
                oversampling = QMC5883L_CONFIG_OS256;
                break;
        case 128:
                oversampling = QMC5883L_CONFIG_OS128;
                break;
        case 64:
                oversampling = QMC5883L_CONFIG_OS64;
                break;
        }
        reconfig();
}

void setRange( int x )
{
        switch(x) {
        case 2:
                range = QMC5883L_CONFIG_2GAUSS;
                DigitPermg=12.0;
                break;
        case 8:
                range = QMC5883L_CONFIG_8GAUSS;
                DigitPermg=3.0;
                break;
        }
        reconfig();
}

void setSamplingRate( int x )
{
        switch(x) {
        case 10:
                rate = QMC5883L_CONFIG_10HZ;
                break;
        case 50:
                rate = QMC5883L_CONFIG_50HZ;
                break;
        case 100:
                rate = QMC5883L_CONFIG_100HZ;
                break;
        case 200:
                rate = QMC5883L_CONFIG_200HZ;
                break;
        }
        reconfig();
}

void QMC5883L_init() {
        /* This assumes the wire library has been initialized. */
        //addr = QMC5883L_ADDR;
        setOversampling(512);
        setRange(2);
        setSamplingRate(100);
        mode = QMC5883L_CONFIG_CONT;
        reset();
}

int readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t )
{
        *x = read_raw_data(QMC5883L_X_LSB);
        *y = read_raw_data(QMC5883L_Y_LSB);
        *z = read_raw_data(QMC5883L_Z_LSB);
        return 1;
}

void resetCalibration() {
        xhigh = yhigh = 0;
        xlow = ylow = 0;
}

int readHeading()
{
        int16_t x, y, z, t;

        if(!readRaw(&x,&y,&z,&t)) return 0;

        /* Update the observed boundaries of the measurements */

        if(x<xlow) xlow = x;
        if(x>xhigh) xhigh = x;
        if(y<ylow) ylow = y;
        if(y>yhigh) yhigh = y;

        /* Bail out if not enough data is available. */

        if( xlow==xhigh || ylow==yhigh ) return 0;

        /* Recenter the measurement by subtracting the average */

        x -= (xhigh+xlow)/2;
        y -= (yhigh+ylow)/2;

        /* Rescale the measurement to the range observed. */

        float fx = (float)x/(xhigh-xlow);
        float fy = (float)y/(yhigh-ylow);

        int heading = 180.0*atan2(fy,fx)/M_PI;
        if(heading<=0) heading += 360;

        return heading;
}

void sigintHandler(int sig)
{
        ros::shutdown();
}


void mag_timer_callback (const ros::TimerEvent&){
        // ROS timer callback to publish mag data
        int16_t x, y, z, t;
        short status;
        sensor_msgs::MagneticField mag_msg;

        status=i2c_read_byte_data(pi,i2ch, QMC5883L_STATUS);
        if(!status & QMC5883L_STATUS_DRDY) {
                return;
        }

        readRaw(&x,&y,&z,&t);
        //ROS_INFO("Raw data %i, %i,%i",x,y,z);
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.header.frame_id="imu_link";
        mag_msg.magnetic_field.x=(int)(x/DigitPermg);
        mag_msg.magnetic_field.y=(int)(y/DigitPermg);
        mag_msg.magnetic_field.z=(int)(z/DigitPermg);
        mag_pub.publish(mag_msg);
}

int main (int argc, char **argv)
{
        ros::init(argc, argv, "pi_car_mag", ros::init_options::NoSigintHandler);
        ROS_INFO("Started Pi Car mag");
        ros::NodeHandle nh;
        signal(SIGINT, sigintHandler);
        signal(SIGTERM, sigintHandler);
        signal(SIGKILL, sigintHandler);
        if (pi=pigpio_start(NULL,NULL)<0) {
                ROS_INFO("gpio inti failed");
                return 1;
        }

        i2ch= i2c_open(pi, 1, QMC5883L_ADDR, 0);
        QMC5883L_init();

        // Set up ROS
        mag_timer = nh.createTimer(ros::Duration(MAG_TIMER), mag_timer_callback);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>(TOPIC_MAG, 2);

        ros::spin();
}
