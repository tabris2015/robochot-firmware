/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <geometry_msgs/Vector3.h>      // for pid tunings
#include <sensor_msgs/JointState.h>     // for feedback
#include "robot_drive.h"

#define PUB_RATE 0.1
#define PID_RATE 0.01

// DigitalOut myled(LED1);
PwmOut led(LED_PIN);

// hardware
Robot robot(1, 0, 0, PID_RATE);

// useful resources and timing
Timer t;
Ticker pub_ticker;

// ros stuff
ros::NodeHandle nh;
char * robot_id = "/robochot";
// ros messages
sensor_msgs::JointState robot_state_msg;

char *names[] = {"L", "R"};
double pos[2];
double vel[2];
double eff[2];
// prototypes
void messageCb(const std_msgs::UInt16 &brightness_msg);
void wheelsCb(const geometry_msgs::Vector3 &wheels_msg);
void pidTuningsCb(const geometry_msgs::Vector3 &pid_tunings_msg);
void pubCb(void);
// -----------------------------------------------

ros::Subscriber<std_msgs::UInt16> led_sub("led_brightness", &messageCb);
ros::Subscriber<geometry_msgs::Vector3> wheels_sub("wheels", &wheelsCb);
ros::Subscriber<geometry_msgs::Vector3> pid_tunings_sub("pid_tunings", &pidTuningsCb);

ros::Publisher robot_state_pub("robot_state", &robot_state_msg);

int main()
{
    
    // ros stuff
    nh.initNode();
    nh.subscribe(led_sub);
    nh.subscribe(pid_tunings_sub);
    nh.subscribe(wheels_sub);
    nh.advertise(robot_state_pub);
    
    robot_state_msg.header.frame_id = robot_id;
    robot_state_msg.name_length = 2;
    robot_state_msg.position_length = 2;
    robot_state_msg.velocity_length = 2;
    robot_state_msg.effort_length = 2;
    robot_state_msg.name = names;
    //setup
    pub_ticker.attach(pubCb, PUB_RATE);
    while (1)
    {
        nh.spinOnce();
    }
}

void messageCb(const std_msgs::UInt16 &brightness_msg)
{
    led = brightness_msg.data / 360.0;
}

void pidTuningsCb(const geometry_msgs::Vector3 &pid_msg)
{
    robot.setPidTunings(pid_msg.x, pid_msg.y, pid_msg.z);
}
void pubCb()
{
    // extract state from robot class
    RobotState state = robot.readState();
    pos[0] = state.l_position;
    pos[1] = state.r_position;
    vel[0] = state.l_speed;
    vel[1] = state.r_speed;
    eff[0] = state.l_effort;
    eff[1] = state.r_effort;

    robot_state_msg.position = pos;
    robot_state_msg.velocity = vel;
    robot_state_msg.effort = eff;
    // publish the message
    robot_state_pub.publish(&robot_state_msg);
}

void wheelsCb(const geometry_msgs::Vector3 &wheels_msg)
{
    robot.setWheels(wheels_msg.x, wheels_msg.y);
}