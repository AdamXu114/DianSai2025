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
#include "TSP_TFT18.h"

int main(void)
{
	uint32_t count=0;
	
	SYSCFG_DL_init();
	
	//DL_FlashCTL_executeClearStatus();
	
	tsp_tft18_init();
	tsp_tft18_test_color();
	tsp_tft18_show_str_color(0, 0, "NUEDC-2025 SAIS@SJTU", BLUE, YELLOW);

	while (1) {
		//delay_1ms(1000);
		//DL_GPIO_togglePins(GPIO_GRP_0_PORT, DL_GPIO_PIN_5);
//		if(S1())
//			LED_ON();
//		else
//			LED_OFF();
		if(S0())
			LED_ON();
		else
			LED_OFF();

//		if(!S1())
//			LCD_BL_ON();
//		else
//			LCD_BL_OFF();

		if(!S2())
			BUZZ_ON();
		else
			BUZZ_OFF();
		
	}	
			  
}
