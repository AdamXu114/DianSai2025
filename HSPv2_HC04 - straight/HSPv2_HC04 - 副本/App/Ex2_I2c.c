#include <stdio.h>
#include "Ex2.h"
#include "systick.h"
#include "math.h"
#include "GFX_FUNCTIONS.h"


int16_t acc_x=0, acc_y=0, acc_z=0;
int16_t acc_x_t=0, acc_y_t=0, acc_z_t=0;
float acc_total=0;

void Ex2_1_seg7()
{
	uint16_t i = 0;

	while(1)
	{
		i++;
		delay_1ms(100);		// delay 0.1 second
		if (PUSH())
			hsp_cat9555_seg7_decimal(i);
		else
			hsp_cat9555_seg7_hexadecimal(i);
	}
}

void Ex2_2_ledbar()
{
	uint16_t i = 0;

	while(1)
	{
		i++;
		delay_1ms(100);		// delay 0.1 second
		
		if(!SW1())
		{
			if (PUSH())
				hsp_cat9555_ledbar(i);
			else
				hsp_cat9555_ledbar(~i);
		}
		else
		{
			if (PUSH())
				hsp_cat9555_seg7_decimal(i);
			else
				hsp_cat9555_seg7_hexadecimal(i);
		}

		if(!S3()) break;
	}
}

void Ex2_3_segbarmux()
{
	uint16_t i=0, j;

	while(1)
	{
		i++;
		j = i>>6;
		hsp_cat9555_seg7_hexadecimal(j);
		delay_1ms(3);		// delay 0.003 second
		hsp_cat9555_ledbar(j);
		delay_1ms(3);		// delay 0.003 second

		if(!S3()) break;
	}
}

void Ex2_4_mems()
{
	uint8_t accl_data[8], temp1;
	uint16_t wx,wy,wz;
	uint8_t x1,x2,y1,y2,z1,z2;

	hsp_tft18_clear(BLACK);

	while(1)
	{
		hsp_mma8451_read_byte(F_STATUS_REG, &temp1);
		if (temp1 & 0x0F)
		{
//			hsp_mma8451_read_byte(OUT_X_MSB_REG, &x1);
//			hsp_mma8451_read_byte(OUT_X_LSB_REG, &x2);
//			hsp_mma8451_read_byte(OUT_Y_MSB_REG, &y1);
//			hsp_mma8451_read_byte(OUT_Y_LSB_REG, &y2);
//			hsp_mma8451_read_byte(OUT_Z_MSB_REG, &z1);
//			hsp_mma8451_read_byte(OUT_Z_LSB_REG, &z2);
//			
//			acc_x = x1<<8;
//			acc_x |= x2;
//			acc_x >>= 2;
//
//			acc_y = y1<<8;
//			acc_y |= y2;
//			acc_y >>= 2;
//
//			acc_z = z1<<8;
//			acc_z |= z2;
//			acc_z >>= 2;
//			printf("X: %d; Y: %d; Z: %d\n\r", acc_x, acc_y, acc_z);

			hsp_mma8451_read_nbyte(OUT_X_MSB_REG, accl_data, 6);
			wx = (uint16_t)accl_data[0]<<8 | accl_data[1];
			if(accl_data[0]>0x7f)
				acc_x = -(int16_t)((~(wx>>2) + 1)&0X3FFF);
			else
				acc_x = (wx>>2) & 0X3FFF;
			wy = (uint16_t)accl_data[2]<<8 | accl_data[3];
			if(accl_data[2]>0x7f)
				acc_y = -(int16_t)((~(wy>>2) + 1)&0X3FFF);
			else
				acc_y = (wy>>2)&0X3FFF;
			wz = (uint16_t)accl_data[4]<<8 | accl_data[5];
			if(accl_data[4]>0x7f)
				acc_z = -(int16_t)((~(wz>>2) + 1)&0X3FFF);
			else
				acc_z = (wz>>2)&0X3FFF;
			//printf("X: %d; Y: %d; Z: %d\n\r", acc_x, acc_y, acc_z);
			acc_total = sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z) \
			  			- sqrt(acc_x_t*acc_x_t + acc_y_t*acc_y_t + acc_z_t*acc_z_t);
			
			hsp_tft18_show_int16_color(8, 0, acc_x, WHITE, BLACK);
			hsp_tft18_show_int16_color(8, 1, acc_y, WHITE, BLACK);
			hsp_tft18_show_int16_color(8, 2, acc_z, WHITE, BLACK);
			
			acc_x_t = acc_x;
			acc_y_t = acc_y;
			acc_z_t = acc_z;
			
			//delay_1ms(10);
		}
		
		if(!S3()) break;
	}
}

void hsp_demo_mems()
{
	uint8_t accl_data[8], temp1;
	uint16_t wx, wy, wz;
	uint16_t bar_value;
	int8_t bar_pos;
	uint8_t x1, x2, y1, y2;
	char value_str[4];

	hsp_tft18_clear(BLACK);
	hsp_tft18_show_str(0, 0, "X:");
	hsp_tft18_show_str(0, 3, "Y:");
	hsp_tft18_show_str(0, 6, "Z:");
	hsp_tft18_draw_block(32, 0, 127, 127, NAVY);
	x1=x2=95; y1=y2=63;
	hsp_cat9555_ledbar(0);

	while(1)
	{
		hsp_mma8451_read_byte(F_STATUS_REG, &temp1);
		if (temp1 & 0x0F)
		{

			hsp_mma8451_read_nbyte(OUT_X_MSB_REG, accl_data, 6);
			
			wx = (uint16_t)accl_data[0]<<8 | accl_data[1];
			if(accl_data[0]>0x7f)
				acc_x = -(int16_t)((~(wx>>2) + 1)&0X3FFF);
			else
				acc_x = (wx>>2) & 0X3FFF;
			
			wy = (uint16_t)accl_data[2]<<8 | accl_data[3];
			if(accl_data[2]>0x7f)
				acc_y = -(int16_t)((~(wy>>2) + 1)&0X3FFF);
			else
				acc_y = (wy>>2)&0X3FFF;
			
			wz = (uint16_t)accl_data[4]<<8 | accl_data[5];
			if(accl_data[4]>0x7f)
				acc_z = -(int16_t)((~(wz>>2) + 1)&0X3FFF);
			else
				acc_z = (wz>>2)&0X3FFF;
			
			if(acc_x >= 0)
			{
				sprintf(value_str, "%04d", acc_x);
				hsp_tft18_show_str_color(0, 1, value_str, RED, BLACK);
			}
			else
			{
				sprintf(value_str, "%04d", -acc_x);
				hsp_tft18_show_str_color(0, 1, value_str, GREEN, BLACK);
			}
			
			if(acc_y >= 0)
			{
				sprintf(value_str, "%04d", acc_y);
				hsp_tft18_show_str_color(0, 4, value_str, RED, BLACK);
			}
			else
			{
				sprintf(value_str, "%04d", -acc_y);
				hsp_tft18_show_str_color(0, 4, value_str, GREEN, BLACK);
			}
			
			if(acc_z >= 0)
			{
				sprintf(value_str, "%04d", acc_z);
				hsp_tft18_show_str_color(0, 7, value_str, RED, BLACK);
			}
			else
			{
				sprintf(value_str, "%04d", -acc_z);
				hsp_tft18_show_str_color(0, 7, value_str, GREEN, BLACK);
			}
			
			if(acc_x > 4096 ) acc_x = 4096;
			if(acc_x < -4096 ) acc_x = -4096;
			x1 = 95 + acc_x*63/4096;		// 32 - 158
			
			if(acc_y > 4096 ) acc_y = 4096;
			if(acc_y < -4096 ) acc_y = -4096;
			//y1 = 63 + acc_y*63/4096;
			y1 = 63 - acc_y*63/4096;		// 0 - 126
			
			// bubble refresh on LCD
			if((x1 != x2) || (y1 != y2))
			{
				hsp_tft18_draw_pixel(x2, y2, NAVY);
				hsp_tft18_draw_pixel(x2, y2+1, NAVY);
				hsp_tft18_draw_pixel(x2+1, y2, NAVY);
				hsp_tft18_draw_pixel(x2+1, y2+1, NAVY);
				hsp_tft18_draw_pixel(x1, y1, YELLOW);
				hsp_tft18_draw_pixel(x1, y1+1, YELLOW);
				hsp_tft18_draw_pixel(x1+1, y1, YELLOW);
				hsp_tft18_draw_pixel(x1+1, y1+1, YELLOW);
				x2 = x1;
				y2 = y1;
			}
			
			// bubble refresh on LED bar
			bar_pos = 8 + acc_x*12/4096;
			if(bar_pos > 15) bar_pos = 15;
			if(bar_pos < 0) bar_pos = 0;
			bar_value = 1 << bar_pos;
			hsp_cat9555_ledbar(bar_value);
		}
			  
		if(!S3())
		{
			while(!S3()) {}
			break;
		}
	}
}