#include <stdio.h>
#include <stdint.h>
#include <LowLevelIOInterface.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "HSP_SDCARD.h"
#include "hsp_liball.h"
#include "HSP_GPIO.h"
#include "00_MENU.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD (M_PI/180.0f)
#define RAD2DEG (180.0f/M_PI)

volatile uint32_t sys_tick_counter=0;
volatile uint16_t RES_value;
volatile int16_t encoder_speed;
volatile uint8_t flagg = 1;


#define SERVO_NEUTRAL    1300
#define SERVO_MAX_DELTA   250

/* 使角度保持在 -180..180 */
static float normalize_angle(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 直行控制 (原 logic) */
static void drive_straight_with_servo(float error_deg) {
    const float Kp = 5.0f;
    int16_t delta = (int16_t)(Kp * error_deg);
    if      (delta >  SERVO_MAX_DELTA) delta =  SERVO_MAX_DELTA;
    else if (delta < -SERVO_MAX_DELTA) delta = -SERVO_MAX_DELTA;
    uint16_t pulse = SERVO_NEUTRAL - delta;
    hsp_servo_angle(SERVO3, pulse);
    /* 前进动力 */
    hsp_motor_voltage(MOTORF, 30);
}

// 姿态角变量
float yaw = 0.0f;
float yaw_ref;
// 三次采集总时长约 3 × 66.7 ms = 200 ms
uint32_t dt = 200;
uint8_t flag_20_ms = 0; // 用于标记 20 ms 周期
char buf[32];


extern int16_t acc_x, acc_y, acc_z;
extern int16_t acc_x_t, acc_y_t, acc_z_t;
extern float acc_total;

uint8_t task_id=0;			// TaskID set by 4-bit DIP switch
uint8_t menu_id=0;			// MenuItem ID returned by Menu_Loop()

uint8_t rx_buffer[100];		// usart2 receiving buffer
uint8_t tx_buffer[100];		// usart2 transmitting buffer
uint16_t rx_idx=0, tx_idx=0;



#pragma location = 0X20030000
uint8_t image_raw[22560];
//__no_init uint8_t image_raw[22560][22560];
//__attribute__((aligned(32))) uint8_t image_raw[22560][22560];


char uart6_rx_buf[640];
int uart6_rx_index = 0;



/* configure the NVIC */
void nvic_config(void)
{
	nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
	nvic_irq_enable(SDIO_IRQn, 0, 1);
}

void board_init()
{
	/* configure the NVIC */
	nvic_config();
	
	/* systick timer 1000Hz interrupts, ISR: SysTick_Handler() */
	systick_config();

	/* GPIO interface such as button, switch, led, buzz, etc */
	hsp_gpio_init();
	
        
        //hsp_resint_init();		// init rotary switch interrupts

	/* I2C interface for CAT9555 */
	hsp_cat9555_init();		// hsp_cat9555_init() included
	
	/* SPI interface for LCD */
	hsp_spi_init();
	hsp_tft18_init();
	//LCD_BL_ON();

	/* MT9V034 */
	hsp_mt9v034_init();
	hsp_dci_dma_config();
	
	/* UART interface for OpenSDA, wireless module, OpenMV, K210 */
	hsp_uart_init();
	hsp_usart1_config();
	hsp_usart2_config();
	hsp_usart5_config();
	hsp_uart6_config();
	
	hsp_usart2_dma_config();
	
	/* PIT periodical interrupt timer, for sensor refresh or PID algorithm */
	//hsp_pit_config();

	// initialize PWM channels for motor and r/c servos
	hsp_pwm_init();
	
	/* optical encoder pulse counter for motor speed feedback */
	//hsp_counter_init();
	hsp_qdec_init();
	
	/* MEMS MMA8451 configuration */
	hsp_i2c_mma8451_init();
	
	/* init all ADC channels */
	hsp_adc_init();
	hsp_adc0_config();
	//hsp_adc1_config();
	//hsp_adc2_config();
}

int main(void)
{
    board_init();
    MEN_HIGH();
    flagg = 1;
    // 初始化软 I2C + MPU6050
    hsp_soft_i2c_init();
    hsp_mpu6050_init_on_PE2_PE3();
    
    
    hsp_servo_angle(SERVO3, 1300);


    /* 初始化 yaw_ref */
    float yaw = 0.0f, yaw_ref;
    {
        float tmp = 0.0f;
        const int N0 = 5;
        uint32_t start_time = get_systick_counter();
        for (int i = 0; i < N0; i++) {
            short gyro[3];
            MPU6050_ReadGyro(gyro);
            int16_t gz = gyro[2];
            uint32_t Current_time = get_systick_counter();
            dt = Current_time - start_time;
            tmp += (gz / 131.0f) * dt / 20.0f / 81.0f * 90.0f / 91.5f * 90.0f;
            start_time = Current_time;
            delay_1ms(3);
        }
        yaw_ref = tmp / N0;
    }

    /* 转向状态相关 */
    uint8_t turning = 0;         // 转向模式标志
    float yaw_target = 0.0f;     // 目标航向
    const float turn_tol = 2.0f; // 允许误差 ±2°

    char buf[32];

   while (1) {
        /* 1. 采样并更新 yaw */
        float tmp = 0.0f;
        if(flag_20_ms) {
            flag_20_ms = 0; // 清除标志
            for (int i = 0; i < 3; i++) {
                short gyro[3];
                MPU6050_ReadGyro(gyro);
                int16_t gz = gyro[2];
                tmp += (gz / 131.0f) ;
            }
        }
        yaw += tmp / 3.0f * 0.020f / 9.5f *90.0f ; // 20 ms, 3 次采样
        yaw = normalize_angle(yaw);


        /* 2. 检测是否要进入 90° 转向 */
        if (!turning && flagg == 1) {
            turning = 1;
            /* 设定目标角 = 当前参考 + 90° */
            yaw_target = normalize_angle(yaw_ref + 90.0f);
        }

        if (turning) {
            /* 3. 转向模式：打死方向舵机，不前进 */
            /* 这里我们让舵机偏到最大以最快速转向 */
            hsp_servo_angle(SERVO3, SERVO_NEUTRAL + SERVO_MAX_DELTA);
            hsp_motor_voltage(MOTORF, 30);

            /* 4. 检查是否到位 */
            float err_turn = normalize_angle(yaw - yaw_target);
            if (fabsf(err_turn) < turn_tol) {
                /* 转向完成 */
                turning = 0;
                flagg = 0;               // 清除外部触发
                yaw_ref = yaw_target;   // 更新新的直行参考
                /* 重置舵机到中立 */
                hsp_servo_angle(SERVO3, SERVO_NEUTRAL);
            }
        }
        else {
            /* 5. 普通直行闭环 */
            float err = normalize_angle(yaw - yaw_ref);
            drive_straight_with_servo(err);
        }

        /* 6. 显示 */
        sprintf(buf, "Yaw:%6.1f", yaw);
        hsp_tft18_show_str(0, 0, buf);
        sprintf(buf, "Ref:%6.1f", yaw_ref);
        hsp_tft18_show_str(0, 1, buf);
        sprintf(buf, "dt:%3d", dt);
        hsp_tft18_show_str(0, 3, buf);
        if (turning) {
            sprintf(buf, "Turn->%6.1f", yaw_target);
            hsp_tft18_show_str(0, 2, buf);
        }
    }
}