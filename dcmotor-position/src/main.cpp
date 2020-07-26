/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include "Motor.h"
#include "QEI.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>

#define DATA_LEN 3
#define MOTOR_PPR 1496

#ifdef F303K8
    #define PWM_PIN PA_8
    #define IN1_PIN PA_11
    #define IN2_PIN PB_5
    #define ENCA_PIN PF_0
    #define ENCB_PIN PF_1
#else
    #define PWM_PIN PC_9
    #define IN1_PIN PB_8
    #define IN2_PIN PB_9
    #define ENCA_PIN PC_8
    #define ENCB_PIN PC_6
#endif

// hardware
// DigitalOut myled(LED1);
PwmOut led(LED1);
// motor
int setpoint = 0;
Motor m1(PWM_PIN, IN1_PIN, IN2_PIN);
// encoder
QEI enc1(ENCA_PIN, ENCB_PIN, NC, MOTOR_PPR, QEI::X4_ENCODING);
// ros stuff
ros::NodeHandle nh;
std_msgs::Int32 pulses_msg;
std_msgs::Float32MultiArray monitoring_msg;

int monitor_data[3];


void messageCb(const std_msgs::UInt16 &brightness_msg)
{
    led = brightness_msg.data / 360.0;
}

void speedCb(const std_msgs::Float32 &speed_msg)
{
    m1.speed(speed_msg.data);
}

void setpointCb(const std_msgs::Int32 &setpoint_msg)
{
    setpoint = setpoint_msg.data;
    monitor_data[0] = setpoint;
}
ros::Subscriber<std_msgs::UInt16> led_sub("led_brightness", &messageCb);
ros::Subscriber<std_msgs::Float32> speed_sub("motor_speed", &speedCb);
ros::Subscriber<std_msgs::Int32> setpoint_sub("setpoint", &setpointCb);

ros::Publisher pulses_pub("motor_pulses", &pulses_msg);
ros::Publisher monitoring_pub("monitoring", &monitoring_msg);

// useful resources and timing
Timer t;

int main()
{
    // init hardware
    m1.period(0.00005);
    enc1.reset();
    nh.initNode();
    // ros stuff
    nh.subscribe(led_sub);
    nh.subscribe(speed_sub);
    nh.advertise(monitoring_pub);

    monitoring_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    monitoring_msg.layout.dim[0].label = "motor";
    monitoring_msg.layout.dim[0].size = 3;
    monitoring_msg.layout.dim[0].stride = 1*3;
    monitoring_msg.layout.data_offset = 0;
    monitoring_msg.data = (std_msgs::Float32MultiArray::_data_type*)malloc(DATA_LEN * sizeof(std_msgs::Float32MultiArray::_data_type));
    
    monitoring_msg.data[0] = (float)setpoint;
    monitoring_msg.data[2] = 0;     //? used for controller effort
    
    //timing
    t.start();
    while (1)
    {
        if(t.read_ms() >= 100)
        {
            t.reset();
            monitoring_msg.data[1] = (float)enc1.getPulses();
            monitoring_pub.publish(&monitoring_msg);
        }
        nh.spinOnce();
    }
}