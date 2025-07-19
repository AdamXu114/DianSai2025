#include <stdio.h>
#include <stdint.h>
#include <LowLevelIOInterface.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "HSP_SDCARD.h"
#include "hsp_liball.h"
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
double gz_avg_all=0;
double gz_bias=0; // MPU6050零点偏差

extern int16_t acc_x, acc_y, acc_z;
extern int16_t acc_x_t, acc_y_t, acc_z_t;
extern float acc_total;

uint8_t task_id=0;			// TaskID set by 4-bit DIP switch
uint8_t menu_id=0;			// MenuItem ID returned by Menu_Loop()

uint8_t rx_buffer[100];		// usart2 receiving buffer
uint8_t tx_buffer[100];		// usart2 transmitting buffer
uint16_t rx_idx=0, tx_idx=0;
double gz;
int32_t n=1;


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

/* HSP board initialize */
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
	//hsp_mt9v034_init();
	//hsp_dci_dma_config();
	
	/* UART interface for OpenSDA, wireless module, OpenMV, K210 */
	//hsp_uart_init();
	//hsp_usart1_config();
	//hsp_usart2_config();
	hsp_usart5_config();
	hsp_uart6_config();
	
	hsp_usart2_dma_config();
	
	/* PIT periodical interrupt timer, for sensor refresh or PID algorithm */
	//hsp_pit_config();

	// initialize PWM channels for motor and r/c servos
	//hsp_pwm_init();
	
	/* optical encoder pulse counter for motor speed feedback */
	//hsp_counter_init();
	//hsp_qdec_init();
	
	/* MEMS MMA8451 configuration */
	hsp_i2c_mma8451_init();
	
	/* init all ADC channels */
	//hsp_adc_init();
	//hsp_adc0_config();
	//hsp_adc1_config();
	//hsp_adc2_config();
}


int main(void)
{
	board_init();
	hsp_soft_i2c_init();
	hsp_mpu6050_init_on_PE2_PE3();

	// 零点偏差校准
	int calib_samples = 1000;
	double calib_sum = 0;
	for (int i = 0; i < calib_samples; i++) {
		int16_t gz_raw = (mpu6050_read_reg(0x47) << 8) | mpu6050_read_reg(0x48);
		calib_sum += gz_raw;
		delay_1ms(10); // 校准采样间隔
	}
	gz_bias = calib_sum / calib_samples;


	// ...existing code...
	float yaw = 0.0f;
	const float dt = 0.2f;
	char buf[32];

	while (1) {
		gz = (mpu6050_read_reg(0x47) << 8) | mpu6050_read_reg(0x48);
		// ...existing code...
		double tmp = (gz + gz_avg_all*(n-1))/n;
		gz_avg_all = tmp;
		n++;
		// 15 Hz 采样间隔 ≈ 66 ms
		delay_1ms(66);
		hsp_tft18_show_int16(0, 3, gz_avg_all);
	}
}
