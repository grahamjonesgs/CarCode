

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "pigpiod_if2.h"

#define RED_LED_PIN 23
#define BLUE_LED_PIN 24
#define SW_RED 11
#define SW_YELLOW 5
int pi;
int command_sent=0;

void delay(int number_of_nano_seconds)
{
        int milli_seconds = 1000 * number_of_nano_seconds;
        clock_t start_time = clock();
        while (clock() < start_time + milli_seconds)
                ;
}

void switch_callback(int pi,uint32_t gpio, uint32_t level, uint32_t tick) {
        int red = gpio_read(pi,SW_RED);
        int yellow = gpio_read(pi,SW_YELLOW);

        if (command_sent==1) {
                return;          // Once we are rebooting or shutting down do nothing more
        }

        if (yellow==PI_LOW&&red==PI_LOW) {
                printf("Shutting down\n");
                command_sent=1;
                for (int i=0; i<10; i++) {
                        gpio_write(pi,RED_LED_PIN,PI_LOW);
                        usleep(1000*200);
                        gpio_write(pi,RED_LED_PIN,PI_HIGH);
                        usleep(1000*200);
                }
                system("shutdown -h now");
                return;
        }

        if (yellow==PI_LOW) {
                printf("Rebooting\n");
                for (int i=0; i<5; i++) {
                        gpio_write(pi,RED_LED_PIN,PI_LOW);
                        usleep(1000*500);
                        gpio_write(pi,RED_LED_PIN,PI_HIGH);
                        usleep(1000*500);
                }
                system("shutdown -r now");
                command_sent=1;
        }
}

int main (int argc, char **argv)
{

        pi=pigpio_start(NULL,NULL);
        if (pi<0) {

                return 1;
        }
        else {
                printf("gpio init ok2\n");

        }

        set_mode(pi,RED_LED_PIN,  PI_OUTPUT);
        set_mode(pi,BLUE_LED_PIN,  PI_OUTPUT);
        set_mode(pi,SW_RED,  PI_INPUT);
        set_mode(pi,SW_YELLOW,  PI_INPUT);

        gpio_write(pi,RED_LED_PIN,PI_HIGH);
        gpio_write(pi,BLUE_LED_PIN,PI_LOW);
        set_pull_up_down(pi, SW_RED, PI_PUD_UP);
        set_pull_up_down(pi, SW_YELLOW, PI_PUD_UP);

        callback(pi, SW_RED, FALLING_EDGE, switch_callback);
        callback(pi, SW_YELLOW, FALLING_EDGE, switch_callback);

        while(1) {
              usleep(1000*500);
        }






}
