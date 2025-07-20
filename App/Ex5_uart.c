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

// USART2 connect to DAP Link: 512000,n,8,1
void Ex5_6_uart2_RGB(void)
{
	uint8_t i = 0;
	uint8_t temp = 0, charx = 0, chary = 0;
	uint8_t red_value=0, green_value=0, blue_value=0;
	uint8_t cmd_buff[12];
	uint8_t cmd_start = 0, cmd_valid = 0, cmd_char_count = 0;
	
	hsp_tft18_clear(BLACK);
	
	hsp_tft18_show_str_color(0, 0, " R ", RED, GRAY0);
	hsp_tft18_show_str_color(56, 0, " G ", GREEN, GRAY0);
	hsp_tft18_show_str_color(112, 0, " B ", BLUE, GRAY0);

	hsp_tft18_show_uint8_color(16, 0, red_value, RED, BLACK);
	hsp_tft18_show_uint8_color(72, 0, green_value, GREEN, BLACK);
	hsp_tft18_show_uint8_color(128, 0, blue_value, BLUE, BLACK);

	hsp_tft18_draw_frame(0, 16, 48, 16, WHITE);
	hsp_tft18_draw_frame(55, 16, 48, 16, WHITE);
	hsp_tft18_draw_frame(111, 16, 48, 16, WHITE);
	hsp_tft18_draw_block(1, 17, 46, 14, COLOR565(red_value, 0, 0));
	hsp_tft18_draw_block(56, 17, 46, 14, COLOR565(0, green_value, 0));
	hsp_tft18_draw_block(112, 17, 46, 14, COLOR565(0, 0, blue_value));

	hsp_tft18_draw_frame(0, 48, 159, 79, WHITE);
	hsp_tft18_draw_block(1, 49, 158, 78, COLOR565(red_value, green_value, blue_value));
	
	while(1)
	{
		if(tx_idx != rx_idx)
		{
			temp = rx_buffer[tx_idx++];
			tx_idx %= 100;
			usart_data_transmit(USART2, temp);
			
			if (temp == 0x2AU)		// '*': 0x2A; '#': 0x23
			{
			  	cmd_start = 1;
				cmd_valid = 0;
				cmd_char_count = 0;
			}
			else if((cmd_start==1) && (cmd_char_count<5))
			{
				cmd_buff[cmd_char_count++] = temp;
			}
			else
			{
				cmd_start = 0;
				cmd_valid = 0;
				cmd_char_count = 0;
			}
			
			if((cmd_char_count==5) && (cmd_buff[4]==0x23U))		// valid ending ¡®#¡¯
			{
				cmd_valid = 1;
				switch(cmd_buff[0])
				{
				case 'R':
				case 'r':
					red_value = (cmd_buff[1]-0x30U)*100 + (cmd_buff[2]-0x30U)*10 + (cmd_buff[3]-0x30U);
					break;
				case 'G':
				case 'g':
					green_value = (cmd_buff[1]-0x30U)*100 + (cmd_buff[2]-0x30U)*10 + (cmd_buff[3]-0x30U);
					break;
				case 'B':
				case 'b':
					blue_value = (cmd_buff[1]-0x30U)*100 + (cmd_buff[2]-0x30U)*10 + (cmd_buff[3]-0x30U);
					break;
				default:		// all invalid cases
					break;
				}
				hsp_tft18_show_uint8_color(16, 0, red_value, RED, BLACK);
				hsp_tft18_show_uint8_color(72, 0, green_value, GREEN, BLACK);
				hsp_tft18_show_uint8_color(128, 0, blue_value, BLUE, BLACK);

				hsp_tft18_draw_block(1, 17, 46, 14, COLOR565(red_value, 0, 0));
				hsp_tft18_draw_block(56, 17, 46, 14, COLOR565(0, green_value, 0));
				hsp_tft18_draw_block(112, 17, 46, 14, COLOR565(0, 0, blue_value));

				hsp_tft18_draw_frame(0, 48, 160, 80, WHITE);
				hsp_tft18_draw_block(1, 49, 158, 78, COLOR565(red_value, green_value, blue_value));
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
