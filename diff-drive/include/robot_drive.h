#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include "mbed.h"
#include "rtos.h"
#include "rtos/Thread.h"
#include "Motor.h"
#include "QEI.h"
#include "PID.h"
#include "config.h"

#define PWM_PERIOD 0.0005
#define L_MOTOR_MAX_SPEED 5.3       // rad/s
#define L_MOTOR_MIN_SPEED -5.3       // rad/s
#define R_MOTOR_MAX_SPEED 5.3       // rad/s
#define R_MOTOR_MIN_SPEED -5.3       // rad/s


struct RobotState
{
    int32_t l_ticks;
    int32_t r_ticks;
    float l_position;
    float r_position;
    float l_speed;
    float r_speed;
    float l_effort;
    float r_effort;
};

struct RobotOdometry
{
    float x_pos;
    float y_pos;
    float theta;
    float v;
    float w;
};
class Robot
{
private:
    // pid stuff
    float kp_;
    float kd_;
    float ki_;
    float pid_rate_;
    uint32_t sample_time_;
    Motor l_motor_;
    Motor r_motor_;
    QEI l_encoder_;
    QEI r_encoder_;
    PID l_pid_;
    PID r_pid_;
    Ticker pid_ticker_;
    Thread control_th_;
    PwmOut * led_ptr_;
    // robot 
    RobotState state_;
    RobotOdometry odom_;

    void controlLoop();
    void updatePid();
    void updateOdometry(int32_t dl_ticks, int32_t dr_ticks);
public:
    Robot(float kp, float kd, float ki, float pid_rate, PwmOut * led_ptr);
    void start();
    void setWheels(float left_speed, float right_speed);
    void setUnicycle(float v, float w);
    RobotState readState();
    RobotOdometry readOdometry();
    void setPidTunings(float kp, float kd, float ki);
};
#endif