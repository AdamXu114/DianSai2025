#include <stdio.h>
#include <string.h>
#include "systick.h"
#include "Ex3.h"
#include "Ex5.h"

uint8_t string_tx[200];
extern uint8_t rx_buffer[];		// usart2 receiving buffer
extern uint8_t tx_buffer[];		// usart2 transmitting buffer
extern uint16_t rx_idx, tx_idx;

// USART1 connect to Open-MV connector J4: 115200,n,8,1
void Ex5_1_uart1(void)
{
	uint16_t i = 0;

	while(1)
	{
		i++;
		sprintf(string_tx, "Loop counter: %08d\n\r", i);
		delay_1ms(100);
	}
}

// USART2 connect to DAP Link: 512000,n,8,1
void Ex5_2_uart2_opensda(void)
{
	uint8_t i = 0;
	uint8_t temp = 0, charx = 0, chary = 0;

	hsp_tft18_clear(BLACK);

	while(1)
	{
		if(SET == usart_flag_get(USART2, USART_FLAG_RBNE))
		{
			temp = usart_data_receive(USART2);
			if ((temp >= 0x20U) && (temp <= 0x7EU))
			{
				charx = (i%20)*8;
				chary = (i/20)*16;
				hsp_tft18_show_char(charx, chary, temp);
				i++;
				if(i >= 160)
				{
					i = 0;
					hsp_tft18_clear(BLACK);
				}
			}
		}

		if (!S3())    // push button pressed        
		{
			delay_1ms(50);  // de-jitter
			if (!S3())
			{
				while(!S3());
				break;
			}
		}
	}
}

// USART5 connect to wireless module HC-04: 9600,n,8,1
void Ex5_3_uart5_wireless(void)
{
	uint8_t i = 0;
	uint8_t temp = 0, charx = 0, chary = 0;

	hsp_tft18_clear(BLACK);

	// show chars from HC-06 on 20*8 text window
	while(1)
	{
		if(SET == usart_flag_get(USART5, USART_FLAG_RBNE))
		{
			temp = usart_data_receive(USART5);
			if ((temp >= 0x20U) && (temp <= 0x7EU))
			{
				charx = (i%20)*8;
				chary = (i/20)*16;
				hsp_tft18_show_char(charx, chary, temp);
				i++;
				if(i >= 160)
					i = 0;
			}
		}

		if (!S3())    // push button pressed        
		{
			delay_1ms(50);  // de-jitter
			if (!S3())
			{
				while(!S3());
				break;
			}
		}
	}
}

// UART6 connect to K210 connector J11: 115200,n,8,1
void Ex5_4_uart6(void)
{
	uint16_t i = 0;

	while(1)
	{
		i++;
		sprintf(string_tx, "Loop counter: %08d\n\r", i);
		delay_1ms(100);
	}
}

// USART2 connect to DAP Link: 512000,n,8,1
void Ex5_5_uart2_irq(void)
{
	uint8_t i = 0;
	uint8_t temp = 0, charx = 0, chary = 0;

	hsp_tft18_clear(BLACK);

	while(1)
	{
		if(tx_idx != rx_idx)
		{
			temp = rx_buffer[tx_idx++];
			tx_idx %= 100;

			if ((temp >= 0x20U) && (temp <= 0x7EU))
			{
				charx = (i%20)*8;
				chary = (i/20)*16;
				hsp_tft18_show_char(charx, chary, temp);
				i++;
				if(i >= 160)
				{
					i = 0;
					hsp_tft18_clear(BLACK);
				}
			}
		}
		
		if (!S3())    // push button pressed        
		{
			delay_1ms(50);  // de-jitter
			if (!S3())
			{
				while(!S3());
				break;
			}
		}
	}
}