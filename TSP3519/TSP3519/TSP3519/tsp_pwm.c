#include "ti_msp_dl_config.h"
#include "tsp_pwm.h"
#include "tsp_tft18.h"

#define CH1_LOWER_LIMIT	1100U
#define CH1_UPPER_LIMIT	1900U
#define CH2_LOWER_LIMIT	1100U
#define CH2_UPPER_LIMIT	1900U

#define MOTOR_DC_LIMIT	60

extern float kp_motor;
extern float ki_motor;
extern float kd_motor; // 电机控制的微分系数
extern float kp_servo; // 舵机控制的比例系数
extern float ki_servo; // 舵机控制的积分系数
extern float kd_servo; // 舵机控制的微分系数
extern float kp_turn_motor; // 原地转向电机控制的比例系数
extern float ki_turn_motor; // 原地转向电机控制的积分系数
extern float kd_turn_motor; // 原地转向电机控制的微分系数

extern float kp_angle_to_err; // 角度转误差的比例系数


void pwm_init(void){
   // SYSCFG_DL_Servo_init();
    // SYSCFG_DL_Motor_init();
}

// 舵机驱动函数
void tsp_servo_angle(uint8_t channel, uint16_t pulse_width){

	uint16_t duty;
	
	duty = pulse_width;
	switch (channel)
	{
		case SERVO1:
			if (CH1_LOWER_LIMIT > pulse_width)
				duty = CH1_LOWER_LIMIT;
			if (CH1_UPPER_LIMIT < pulse_width)
				duty = CH1_UPPER_LIMIT;
			DL_TimerG_setCaptureCompareValue(Servo_INST, duty, DL_TIMER_CC_0_INDEX);
            tsp_tft18_show_str_color(0, 1, "Servo1 Angle Set", BLUE, YELLOW);
			break;
		case SERVO2:
			if (CH2_LOWER_LIMIT > pulse_width)
				duty = CH2_LOWER_LIMIT;
			if (CH2_UPPER_LIMIT < pulse_width)
				duty = CH2_UPPER_LIMIT;
			DL_TimerG_setCaptureCompareValue(Servo_INST, duty, DL_TIMER_CC_1_INDEX);
			break;
		default:
			break;
	}
}

// 电机驱动函数
void tsp_motor_voltage(uint8_t dir, uint16_t duty_cycle)
{
	uint16_t dc;
	
	dc = duty_cycle;
	if (MOTOR_DC_LIMIT < duty_cycle)
		dc = MOTOR_DC_LIMIT;
	
	switch (dir)
	{
		case MOTORF:
			DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_0_INDEX);
			DL_TimerG_setCaptureCompareValue(Motor_INST, dc, DL_TIMER_CC_1_INDEX);
			break;
		case MOTORB:
			DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_0_INDEX);
			DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_1_INDEX);
			break;
		default:
			break;
	}
}

void tsp_motor_stop(void) // 停车
{
	DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_0_INDEX);
	DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_1_INDEX);
	tsp_tft18_show_str_color(0, 1, "Motor Stop", BLUE, YELLOW);
}

// 实现原地左转或右转一定角度的函数
void tsp_motor_turn_inplace(uint8_t dir, uint16_t duty_cycle, uint16_t angle) 
{
	uint16_t dc;
	
	dc = duty_cycle;
	if (MOTOR_DC_LIMIT < duty_cycle)
		dc = MOTOR_DC_LIMIT;
	float target_yaw = yaw;
	switch(dir){
		case LEFT:
		target_yaw = yaw + angle; // yaw是当前角度，需要获取当前yaw，下面的yaw也待更新
		break;
		case RIGHT:
		target_yaw = yaw - angle; // yaw是当前角度，需要获取当前yaw，下面的yaw也待更新
		break;
		default:
			return; // 无效方向
	}
	
	if (target_yaw > 360.0f) target_yaw -= 360.0f;
	if (target_yaw < 0.0f) target_yaw += 360.0f;

	tsp_tft18_show_str_color(0, 1, "Motor Turn Left", BLUE, YELLOW);

	// 简单PID参数
	float error, output;
	while (1) {
		// 计算误差
		error = target_yaw - yaw;
		// 保证误差在[-180,180]区间
		if (error > 180.0f) error -= 360.0f;
		if (error < -180.0f) error += 360.0f;

		// 到达目标角度则退出
		if (fabsf(error) < 2.0f) break;

		// 简单P控制
		output = kp_turn_motor * error;
		if (output > dc) output = dc;
		if (output < 10) output = 10; // 最小转速

		DL_TimerG_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_0_INDEX);
		DL_TimerG_setCaptureCompareValue(Motor_INST, (uint16_t)output, DL_TIMER_CC_1_INDEX);

		// 这里应有延时和yaw更新
		tsp_delay_ms(10);
		yaw = get_current_yaw(); // 需要实现获取当前yaw的函数
	}
}

// PID控制电机速度
void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor)
{
	static float integral = 0.0f;
	static float prev_error = 0.0f;
	float error = target_speed_pid - current_speed;
	float output;

	integral += error;
	float derivative = error - prev_error;

	output = kp_motor * error + ki_motor * integral + kd_motor * derivative;

	// 限制输出范围
	if (output > MOTOR_DC_LIMIT) output = MOTOR_DC_LIMIT;
	if (output < 0) output = 0;

	// 设置PWM占空比
	if (motor == MOTOR1)
		DL_TimerG_setCaptureCompareValue(Motor_INST, (uint16_t)output, DL_TIMER_CC_0_INDEX);
	else if (motor == MOTOR2)
		DL_TimerG_setCaptureCompareValue(Motor_INST, (uint16_t)output, DL_TIMER_CC_1_INDEX);
	else
		return; // 无效电机编号

	prev_error = error;
}

// 差速底盘巡线
void tsp_line_follower(float err){ 
	tsp_motor_speed_pid(target_speed + kp_angle_to_err * err, MOTOR1);
	tsp_motor_speed_pid(target_speed - kp_angle_to_err * err, MOTOR2);
}

