#ifndef TSP_PWM_H
#define TSP_PWM_H

#include <stdint.h>

// 舵机通道定义
#define SERVO1  0
#define SERVO2  1

// 电机方向定义
#define MOTORF  0 // 前进
#define MOTORB  1 // 后退

#define MOTOR1  0
#define MOTOR2  1

#define LEFT  0
#define RIGHT 1

// PID参数变量声明
extern float kp_motor;
extern float ki_motor;
extern float kd_motor;
extern float kp_servo;
extern float ki_servo;
extern float kd_servo;
extern float kp_turn_motor;
extern float ki_turn_motor;
extern float kd_turn_motor;
extern float kp_angle_to_err;

// 其他外部变量声明（如yaw、current_speed、target_speed等，若在c文件有用到且未在本头文件声明，可补充）
extern float yaw;
extern float current_speed;
extern float target_speed;

// 函数声明
void pwm_init(void);
void tsp_servo_angle(uint8_t channel, uint16_t pulse_width);
void tsp_motor_voltage(uint8_t dir, uint16_t duty_cycle);
void tsp_motor_stop(void);
void tsp_motor_turn_inplace(uint8_t dir, uint16_t duty_cycle, uint16_t angle);
void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor);
void tsp_line_follower(float err);

#endif // TSP_PWM_H
