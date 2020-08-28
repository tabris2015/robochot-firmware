#include "robot_drive.h"

Robot::Robot(float kp, float kd, float ki,float pid_rate, PwmOut * led_ptr):
kp_(kp), kd_(kd), ki_(ki), pid_rate_(pid_rate), sample_time_((uint32_t)(pid_rate_ * 1000.0)),
l_motor_(L_PWM_PIN, L_IN1_PIN, L_IN2_PIN),
r_motor_(R_PWM_PIN, R_IN1_PIN, R_IN2_PIN),
l_encoder_(L_ENCA_PIN, L_ENCB_PIN, NC, MOTOR_PPR, QEI::X4_ENCODING),
r_encoder_(R_ENCA_PIN, R_ENCB_PIN, NC, MOTOR_PPR, QEI::X4_ENCODING),
l_pid_(kp_, kd_, ki_, pid_rate_),
r_pid_(kp_, kd_, ki_, pid_rate_),
control_th_(osPriorityAboveNormal),
led_ptr_(led_ptr),
odom_({0.0, 0.0, 0.0, 0.0, 0.0})
{
    // init robot
    l_motor_.period(PWM_PERIOD);
    r_motor_.period(PWM_PERIOD);
    l_motor_.brake();
    r_motor_.brake();
    l_encoder_.reset();
    r_encoder_.reset();

    // pid
    l_pid_.setInputLimits(L_MOTOR_MIN_SPEED, L_MOTOR_MAX_SPEED);
    r_pid_.setInputLimits(R_MOTOR_MIN_SPEED, R_MOTOR_MAX_SPEED);
    l_pid_.setOutputLimits(-1.0, 1.0);
    r_pid_.setOutputLimits(-1.0, 1.0);
    l_pid_.setBias(0.0);
    r_pid_.setBias(0.0);
    l_pid_.setSetPoint(0.0);
    r_pid_.setSetPoint(0.0);
}

void Robot::start()
{
    control_th_.start(callback(this, &Robot::controlLoop));
}

void Robot::controlLoop()
{
    while (true)
    {   
        uint64_t next = Kernel::get_ms_count() + sample_time_;
        updatePid();
        ThisThread::sleep_until(next);
    }
    
}

void Robot::updatePid()
{
    int32_t l_ticks = l_encoder_.getPulses();
    int32_t r_ticks = r_encoder_.getPulses();
    
    state_.l_position = (2.0 * M_PI) * l_ticks / MOTOR_PPR;
    state_.r_position = (2.0 * M_PI) * r_ticks / MOTOR_PPR;

    int32_t dl_ticks = l_ticks - state_.l_ticks;
    int32_t dr_ticks = r_ticks - state_.r_ticks;

    // update odometry
    updateOdometry(dl_ticks, dr_ticks);
    //

    state_.l_speed = (2.0 * M_PI) * dl_ticks / (MOTOR_PPR * pid_rate_);
    state_.r_speed = (2.0 * M_PI) * dr_ticks / (MOTOR_PPR * pid_rate_);
    
    odom_.v = (WHEEL_RADIUS / 2) * (state_.l_speed + state_.r_speed);
    odom_.w = (WHEEL_RADIUS / WHEEL_SEPARATION) * (state_.r_speed - state_.l_speed);

    l_pid_.setProcessValue(state_.l_speed);
    r_pid_.setProcessValue(state_.r_speed);
    
    state_.l_effort = l_pid_.compute();
    state_.r_effort = r_pid_.compute();

    l_motor_.speed(state_.l_effort);
    r_motor_.speed(state_.r_effort);

    state_.l_ticks = l_ticks;
    state_.r_ticks = r_ticks;
}

void Robot::updateOdometry(int32_t dl_ticks, int32_t dr_ticks)
{
    float Dl = (2 * M_PI * WHEEL_RADIUS * dl_ticks) / MOTOR_PPR;
    float Dr = (2 * M_PI * WHEEL_RADIUS * dr_ticks) / MOTOR_PPR;
    float Dc = (Dl + Dr) / 2;

    odom_.x_pos += Dc * cos(odom_.theta);
    odom_.y_pos += Dc * sin(odom_.theta);
    odom_.theta += (Dr - Dl) / WHEEL_SEPARATION;
}

void Robot::setWheels(float left_speed, float right_speed)
{
    l_pid_.setSetPoint(left_speed);
    r_pid_.setSetPoint(right_speed);
}

void Robot::setUnicycle(float v, float w)
{
    //              m/s  rad/s  m                       m
    float v_l = (2 * v - w * WHEEL_SEPARATION) / (2 * WHEEL_RADIUS);
    float v_r = (2 * v + w * WHEEL_SEPARATION) / (2 * WHEEL_RADIUS);

    setWheels(v_l, v_r);
}

RobotState Robot::readState()
{
    return state_;
}

RobotOdometry Robot::readOdometry()
{
    return odom_;
}

void Robot::setPidTunings(float kp, float kd, float ki)
{
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;

    l_pid_.setTunings(kp_, kd_, ki_);
    r_pid_.setTunings(kp_, kd_, ki_);
}