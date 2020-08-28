#ifndef CONFIG_H
#define CONFIG_H

#define DATA_LEN 3

// robot defines
#define MOTOR_PPR 10640//1496
#define WHEEL_RADIUS 0.04
#define WHEEL_SEPARATION 0.16

// board defines
#ifdef F303K8
    #define LED_PIN PB_4
    #define PWM_PIN PA_8
    #define IN1_PIN PA_11
    #define IN2_PIN PB_5
    #define ENCA_PIN PF_0
    #define ENCB_PIN PF_1
#else
    #define LED_PIN PA_5
    #define L_PWM_PIN PC_9
    #define L_IN1_PIN PB_8
    #define L_IN2_PIN PB_9
    #define L_ENCA_PIN PC_8
    #define L_ENCB_PIN PC_6
    #define R_PWM_PIN PA_6
    #define R_IN1_PIN PA_7
    #define R_IN2_PIN PB_6
    #define R_ENCA_PIN PA_11
    #define R_ENCB_PIN PB_12

#endif

#endif