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

// 姿态角变量
float yaw = 0.0f;
float yaw_ref;
// 三次采集总时长约 3 × 66.7 ms = 200 ms
const float dt = 0.2f;
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

/* 舵机中立脉宽，建议根据标定值修改 */
#define SERVO_NEUTRAL    1300   // 停转时脉宽 1500µs
/* 最大偏移量：对应舵机最大旋转速度 */
#define SERVO_MAX_DELTA   400   // 最大偏移  ±400µs

/* 用 yaw 误差控制舵机 */
void drive_straight_with_servo(float error_deg) {
    const float Kp = 5.0f;     // P系数（脉宽偏移 µs/°）
    int16_t delta = (int16_t)(Kp * error_deg);

    /* 限幅 */
    if      (delta >  SERVO_MAX_DELTA) delta =  SERVO_MAX_DELTA;
    else if (delta < -SERVO_MAX_DELTA) delta = -SERVO_MAX_DELTA;

    /* 左右舵机分别减/加偏移 */
    uint16_t pulse_left  = SERVO_NEUTRAL - delta;
    // uint16_t pulse_right = SERVO_NEUTRAL + delta;

    /* 输出到舵机 */
    hsp_servo_angle(SERVO3, pulse_left);
  //  hsp_servo_angle(SERVO3, pulse_right);
}



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

    // 初始化软 I2C + MPU6050
    hsp_soft_i2c_init();
    hsp_mpu6050_init_on_PE2_PE3();
    
    
    hsp_servo_angle(SERVO3, 1300);


    /*—— 采集几次陀螺角速度，积分得到初始 yaw_ref ——*/
    {
        float tmp = 0.0f;
        const int N0 = 5;
        for (int i = 0; i < N0; i++) {
            int16_t gz = (mpu6050_read_reg(0x47)<<8) | mpu6050_read_reg(0x48);
            tmp += (gz / 131.0f) * dt;
            delay_1ms(200);
        }
        yaw_ref = tmp / N0;
    }

    char buf[32];

    while (1) {
        // —— 3×15Hz 采样并平均陀螺 Z —— 
        int32_t gz_sum = 0;
        for (int i = 0; i < 3; i++) {
            int16_t gz = (mpu6050_read_reg(0x47)<<8) | mpu6050_read_reg(0x48);
            gz_sum += gz;
            delay_1ms(20);
        }
        float gz_avg = (gz_sum / 3) / 131.0f;  // °/s

        // —— 更新 yaw —— 
        yaw += gz_avg * dt;
        if (yaw > 180.0f)       yaw -= 360.0f;
        else if (yaw < -180.0f) yaw += 360.0f;

        // —— 计算偏差并驱动 —— 
        float error = yaw - yaw_ref;
        drive_straight_with_servo(error);
        hsp_motor_voltage(MOTORF,40);
        // —— 在屏幕上显示 当前 yaw 和 偏差 —— 
        sprintf(buf, "Yaw:%6.1f", yaw);
        hsp_tft18_show_str(0, 0, buf);
        sprintf(buf, "Err:%6.1f", error);
        hsp_tft18_show_str(0, 1, buf);
    }
}