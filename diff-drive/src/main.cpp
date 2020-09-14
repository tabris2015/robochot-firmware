/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include "mbed.h"
#include "rtos/rtos.h"
#include "BNO055.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
// #include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "robot_drive.h"


#define PUB_RATE 0.04
#define PID_RATE 0.01
#define MSG_TIMEOUT 1.0

PwmOut led(LED_PIN);

// hardware
// BNO055 imu(PB_3, PB_10);        // I2C2
Robot robot(4, 0.1, 0, PID_RATE, &led);

// useful resources and timing
Timer inactive_timer;
volatile bool pub_flag = false;
Ticker pub_ticker;
Ticker odom_ticker;

// ros stuff
ros::NodeHandle nh;
char * robot_id = "/robochot";
// ros messages
geometry_msgs::TransformStamped t;
geometry_msgs::TwistStamped odom_twist;


char *names[] = {"L", "R"};
double pos[2];
double vel[2];
double eff[2];
//
char base_link_id[] = "/base_link";
char odom_id[] = "/odom";
// prototypes
void twistCb(const geometry_msgs::Twist &twist_msg);
void odomCb(void);
void pubOdom(void);
//
void initMsg();
// -----------------------------------------------

// tf
tf::TransformBroadcaster broadcaster;
//

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", &twistCb);

// ros::Publisher imu_pub("robot_imu", &imu_msg);
ros::Publisher twist_pub("odom_twist", &odom_twist);

int main()
{
    // robot hardware
    // imu.reset();
    // if (!imu.check())
    //     while (true) {
    //         led = !led;
    //         ThisThread::sleep_for(100);
    //     }
    // imu.setmode(OPERATION_MODE_NDOF);   // fusion mode
    // ros stuff
    nh.initNode();
    nh.subscribe(twist_sub);
    nh.advertise(twist_pub);
    broadcaster.init(nh);

    initMsg();
    // nh.advertise(imu_pub);
    
    //setup
    odom_ticker.attach(odomCb,PUB_RATE);
    robot.start();
    inactive_timer.start();
    while (1)
    {
        if(pub_flag)
        {
            pubOdom();
            pub_flag = false;
        }
        if(inactive_timer.read() >= MSG_TIMEOUT)
        {
            robot.setWheels(0.0, 0.0);
            inactive_timer.reset();
            inactive_timer.stop();
        }
        nh.spinOnce();
    }
}

void initMsg()
{
    // tf
    t.header.frame_id = odom_id;
    t.child_frame_id = base_link_id;
    odom_twist.header.frame_id = base_link_id;
}

void odomCb()
{
    pub_flag = true;
}

void pubOdom()
{
    RobotOdometry odom = robot.readOdometry();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(odom.theta);

    ros::Time now = nh.now();
    // transform
    // t.header.frame_id = odom;
    // t.child_frame_id = base_link;
    t.header.stamp = now;
    t.transform.translation.x = odom.x_pos;
    t.transform.translation.y = odom.y_pos;
    t.transform.translation.z = 0.04;
    t.transform.rotation = odom_quat;

    // odom twist
    odom_twist.header.stamp = now;
    odom_twist.twist.linear.x = odom.v;
    odom_twist.twist.angular.z = odom.w;

    broadcaster.sendTransform(t);
    twist_pub.publish(&odom_twist);
}

void twistCb(const geometry_msgs::Twist &twist_msg)
{
    robot.setUnicycle(twist_msg.linear.x, twist_msg.angular.z);
    inactive_timer.reset();
    inactive_timer.start();
}