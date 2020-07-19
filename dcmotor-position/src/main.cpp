/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include "Motor.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

// hardware
// DigitalOut myled(LED1);
PwmOut led(LED1);
// motors
Motor m1(PC_9, PB_8, PB_9);

// ros stuff
ros::NodeHandle nh;

void messageCb(const std_msgs::UInt16 &brightness_msg)
{
    led = brightness_msg.data / 360.0;
}

void speedCb(const std_msgs::Float32 &speed_msg)
{
    m1.speed(speed_msg.data);
}

ros::Subscriber<std_msgs::UInt16> led_sub("led_brightness", &messageCb);
ros::Subscriber<std_msgs::Float32> speed_sub("motor_speed", &speedCb);


int main()
{
    // init hardware
    m1.period(0.00005);
    nh.initNode();
    nh.subscribe(led_sub);
    nh.subscribe(speed_sub);

    while (1)
    {
        nh.spinOnce();
        wait_ms(1);
    }
}