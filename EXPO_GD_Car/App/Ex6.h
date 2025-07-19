#ifndef _EX6_H
#define _EX6_H

#include "hsp_timer.h"
#include "HSP_MOTOR.h"

void hsp_demo_motor_feedback(void);
void hsp_demo_motor(void);
void hsp_demo_servo(void);
void Ex6_1_servo_sweep(void);
void Ex6_2_servo_manual(void);
void Ex6_3_motor_manual(void);
void Ex6_4_motor_manual(void);
void hsp_demo_frame_servo();
int16_t motor_get_speed(void);

#endif