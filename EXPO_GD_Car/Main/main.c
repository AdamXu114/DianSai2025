/*!
    \file    main.c
    \brief   running led

    \version 2023-06-25, V3.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdint.h>
#include <LowLevelIOInterface.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "HSP_SDCARD.h"
#include "hsp_liball.h"
#include "00_MENU.h"

volatile uint32_t sys_tick_counter=0;
volatile uint16_t RES_value;
volatile int16_t encoder_speed;
volatile uint16_t round_speed; // 车轮转速，单位round per second

extern int16_t acc_x, acc_y, acc_z;
extern int16_t acc_x_t, acc_y_t, acc_z_t;
extern float acc_total;

uint8_t task_id=0;			// TaskID set by 4-bit DIP switch
uint8_t menu_id=0;			// MenuItem ID returned by Menu_Loop()

uint8_t rx_buffer[100];		// usart2 receiving buffer
uint8_t tx_buffer[100];		// usart2 transmitting buffer
uint16_t rx_idx=0, tx_idx=0;
uint8_t rx_flag = 0;		// 接收完成标志

#pragma location = 0X20030000
uint8_t image_raw[22560];
//__no_init uint8_t image_raw[22560][22560];
//__attribute__((aligned(32))) uint8_t image_raw[22560][22560];

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
	hsp_resint_init();		// init rotary switch interrupts

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

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
	uint16_t i;
	uint16_t retry_count = 5;
   
	board_init();

	LED_R_OFF();
	LED_G_OFF();
	LED_B_OFF();

	while(1) {
		// main menu for car demo
		menu_id = hsp_menu_car();

		switch(menu_id)
		{
			case 0U:		// 1. Digital camera vision, with binarization functions
				hsp_demo_dcam();
				break;
			case 1U:		// 2. Image processing demo, edge / erode / dilate / gauss
				hsp_demo_image_process();
				break;
			case 2U:		// 3. R/C servo manual control
			  	hsp_demo_servo();
				break;
			case 3U:		// 4. Motor manual control
			  	hsp_demo_motor();
				break;
			case 4U:		// 5. Line following illustrating by R/C servo
			  	Project_LFR();
				break;
			case 5U:		// 6. Motor closed-loop control
				//hsp_demo_motor_feedback();		//reserved
			  while(!S3()) {}
				break;
			case 6U:		// 7. 3-axis accelerometer
				hsp_demo_mems();
				break;
			case 7U:		// 8: RTC date and time
				hsp_rtc_demo();
				break;
		}
		
		while(!S3()) {}	// wait until S3 released
	}
}