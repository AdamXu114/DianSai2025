/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "tsp_isr.h"
#include "tsp_gpio.h"
#include "tsp_soft_i2c.h"
#include "TSP_soft6050.h"
#include "TSP_TFT18.h"

uint8_t flag_20_ms = 0; // 用于标记 20 ms 周期
uint32_t dt = 200;
static float normalize_angle(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
int16_t AX,AY,AZ,GX,GY,GZ;
int main(void)
{
	uint32_t count=0;
	
	SYSCFG_DL_init();
	
	//DL_FlashCTL_executeClearStatus();
	
	tsp_tft18_init();
	tsp_tft18_test_color();
	tsp_tft18_show_str_color(0, 0, "NUEDC-2025 SAIS@SJTU", BLUE, YELLOW);
    MPU6050_Init();
    
	/* 初始化 yaw_ref */
    float yaw = 0.0f;
    
	while (1) {
		
		if(S0())
			LED_ON();
		else
			LED_OFF();


		if(!S2())
			BUZZ_ON();
		else
			BUZZ_OFF();
		
        
		float tmp = 0.0f;
        if(flag_20_ms) {
            flag_20_ms = 0; // 清除标志
            for (int i = 0; i < 3; i++) {
                MPU6050_GetData(&AX,&AY,&AZ,&GX,&GY,&GZ);
                int16_t gz = GZ;
                //tsp_tft18_show_int16(0, 6, gz);
                tmp += (gz / 131.0f) ;
            }
        }
		char buf[32];
        yaw += tmp / 3.0f * 0.020f / 9.5f *90.0f ; // 20 ms, 3 次采样
        yaw = normalize_angle(yaw);
		sprintf(buf, "Yaw:%6.1f", yaw);
        tsp_tft18_show_str(0, 3, buf);
	}	
			  
}
