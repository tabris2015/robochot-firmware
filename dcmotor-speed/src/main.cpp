/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include "PID.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>      // for pid tunings

#include "config.h"

#define PUB_RATE 0.1
#define PID_RATE 0.01

// hardware
// DigitalOut myled(LED1);
PwmOut led(LED_PIN);
// motor
float setpoint = 0;
float speed = 0;
float kp = 1;
float ki = 0;
float kd = 0;
int last_pulses = 0;
Motor m1(PWM_PIN, IN1_PIN, IN2_PIN);
// encoder
QEI enc1(ENCA_PIN, ENCB_PIN, NC, MOTOR_PPR, QEI::X4_ENCODING);
// pid
PID pid(kp, ki, kd, PID_RATE);
// useful resources and timing
Timer t;
Ticker pub_ticker;
Ticker pid_ticker;

// ros stuff
ros::NodeHandle nh;
std_msgs::Int32 pulses_msg;
// std_msgs::Float32MultiArray monitoring_msg;
geometry_msgs::Vector3 monitoring_msg;
// prototypes
void messageCb(const std_msgs::UInt16 &brightness_msg);
void speedCb(const std_msgs::Float32 &speed_msg);
void setpointCb(const std_msgs::Float32 &setpoint_msg);
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg);
void pubCb(void);
void pidCb(void);
void initPid();
// -----------------------------------------------

ros::Subscriber<std_msgs::UInt16> led_sub("led_brightness", &messageCb);
ros::Subscriber<std_msgs::Float32> speed_sub("motor_speed", &speedCb);
ros::Subscriber<std_msgs::Float32> setpoint_sub("setpoint", &setpointCb);
ros::Subscriber<geometry_msgs::Vector3> pid_tunings_sub("pid_tunings", &pidTuningsCb);

ros::Publisher pulses_pub("motor_pulses", &pulses_msg);
ros::Publisher monitoring_pub("monitoring", &monitoring_msg);

int main()
{
    // init hardware
    m1.period(0.00005);
    enc1.reset();
    // ros stuff
    nh.initNode();
    nh.advertise(monitoring_pub);
    nh.subscribe(led_sub);
    nh.subscribe(setpoint_sub);
    nh.subscribe(pid_tunings_sub);

    // monitoring_msg.data_length = DATA_LEN;
    // monitoring_msg.data = (float *)malloc(sizeof(float)*DATA_LEN);

    // monitoring_msg.data[0] = (float)setpoint;
    monitoring_msg.x = setpoint;

    // monitoring_msg.data[2] = 0;     //? used for controller effort
    
    //setup
    initPid();
    pid.setSetPoint(setpoint);
    pub_ticker.attach(pubCb, PUB_RATE);
    pid_ticker.attach(pidCb, PID_RATE);
    while (1)
    {
        nh.spinOnce();
    }
}

// functions and callbacks

void initPid()
{
    pid.setInputLimits(-5.3, 5.3);
    pid.setOutputLimits(-1.0, 1.0);
    pid.setBias(0.0);
    pid.setMode(AUTO_MODE);
}

void messageCb(const std_msgs::UInt16 &brightness_msg)
{
    led = brightness_msg.data / 360.0;
}

void speedCb(const std_msgs::Float32 &speed_msg)
{
    m1.speed(speed_msg.data);
}

void setpointCb(const std_msgs::Float32 &setpoint_msg)
{
    setpoint = setpoint_msg.data;
    monitoring_msg.x = setpoint;
    pid.setSetPoint(setpoint);
}
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg)
{
    kp = pid_tunings_msg.x;
    ki = pid_tunings_msg.y;
    kd = pid_tunings_msg.z;
    pid.setTunings(kp, ki, kd);
}
void pubCb()
{
    // publish mesages
    // [setpoint, position, effort]
    monitoring_msg.y = speed;
    monitoring_pub.publish(&monitoring_msg);
}

void pidCb()
{
    int pulses = enc1.getPulses();
    int d_pulses = pulses - last_pulses;
    speed = (2.0 * M_PI) * d_pulses / (MOTOR_PPR * PID_RATE);
    pid.setProcessValue(speed);
    float out = pid.compute();
    m1.speed(out);
    last_pulses = pulses;
    monitoring_msg.z = out;
}
