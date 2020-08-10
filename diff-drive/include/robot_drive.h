#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include "mbed.h"
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

class Robot
{
private:
    // pid stuff
    float kp_;
    float kd_;
    float ki_;
    float pid_rate_;
    Motor l_motor_;
    Motor r_motor_;
    QEI l_encoder_;
    QEI r_encoder_;
    PID l_pid_;
    PID r_pid_;
    Ticker pid_ticker_;

    // robot 
    RobotState state_;

    void updatePid();
public:
    Robot(float kp, float kd, float ki, float pid_rate);
    void setWheels(float left_speed, float right_speed);
    RobotState readState();
    void setPidTunings(float kp, float kd, float ki);
};
#endif