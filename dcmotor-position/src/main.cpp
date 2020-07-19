/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>

// hardware
// DigitalOut myled(LED1);
PwmOut led(LED1);

// ros stuff
ros::NodeHandle nh;



void messageCb(const std_msgs::UInt16& brightness_msg){
    led = brightness_msg.data / 360.0;
}

ros::Subscriber<std_msgs::UInt16> sub("led_brightness", &messageCb);

int main() {
    nh.initNode();
    nh.subscribe(sub);

    while (1) {
        nh.spinOnce();
        wait_ms(1);
    }
}