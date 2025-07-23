#ifndef _TSP_UART_H
#define _TSP_UART_H

#include "ti_msp_dl_config.h"



extern void UART6_IRQHandler(void);
extern SYSCONFIG_WEAK void SYSCFG_DL_K230_init(void);

void tsp_uart6_init(void);
void tsp_uart6_receive(void);





#endif

//对文件“iar\ti_msp_dl_config.c”进行了修改