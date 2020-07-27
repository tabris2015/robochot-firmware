#ifndef CONFIG_H
#define CONFIG_H

#define DATA_LEN 3
#define MOTOR_PPR 10640//1496

#ifdef F303K8
    #define LED_PIN PB_4
    #define PWM_PIN PA_8
    #define IN1_PIN PA_11
    #define IN2_PIN PB_5
    #define ENCA_PIN PF_0
    #define ENCB_PIN PF_1
#else
    #define LED_PIN PA_5
    #define PWM_PIN PC_9
    #define IN1_PIN PB_8
    #define IN2_PIN PB_9
    #define ENCA_PIN PC_8
    #define ENCB_PIN PC_6
#endif
#endif