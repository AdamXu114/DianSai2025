#include "tsp_uart.h"
#include <string.h>

extern uint8_t rx_buffer[];
extern uint16_t rx_idx;
extern uint8_t rx_flag;

void tsp_uart6_init(void)
{
    SYSCFG_DL_K230_init();
    // // —— 第一步：复位并配置时钟，初始化 UART 硬件 ——  
    // DL_UART_Main_deinit(K230_INST);  // 如果有复位接口，先复位
    // DL_UART_Main_setClockConfig(K230_INST, (DL_UART_Main_ClockConfig *)&gK230ClockConfig);
    // DL_UART_Main_init(K230_INST, (DL_UART_Main_Config       *)&gK230Config);

    // // —— 第二步：设置波特率 ——  
    // // 目标 115200，实际大约 115190.78  
    // DL_UART_Main_setOversampling(K230_INST, DL_UART_OVERSAMPLING_RATE_16X);
    // DL_UART_Main_setBaudRateDivisor(K230_INST,
    //     K230_IBRD_80_MHZ_115200_BAUD,
    //     K230_FBRD_80_MHZ_115200_BAUD);

    // // // —— 第四步：关闭硬件流控 ——  
    // // DL_UART_Main_setFlowControl  (K230_INST, DL_UART_FLOW_NONE);        // 关闭 RTS/CTS

    // // // —— 第五步：使能收发和 DMA ——  
    // //DL_UART_Main_enableRx        (K230_INST);                           // 使能接收
    // //DL_UART_Main_enableTx        (K230_INST);                           // 使能发送
    // // DL_UART_Main_enableDMA       (K230_INST, DL_UART_DMA_TX_ENABLE);    // 使能发送 DMA

    // // // —— 第六步：清除传输完成标志 ——  
    // // DL_UART_Main_clearFlag       (K230_INST, DL_UART_FLAG_TC);

    // // —— 第七步：配置中断 ——  
    // DL_UART_Main_enableInterrupt (K230_INST, DL_UART_MAIN_INTERRUPT_RXD_POS_EDGE);         // 接收非空中断
    // // DL_UART_Main_disableInterrupt(K230_INST, DL_UART_MAIN_INTERRUPT_IDLE  // 清除空闲中断
    // //                                        | DL_UART_MAIN_INTERRUPT_ERR);  // 清除错误中断
    // NVIC_EnableIRQ    (UART6_IRQHandler);  // 使能 UART6 中断

    // // —— 第八步：最终使能 UART ——  
    // DL_UART_Main_enable       (K230_INST);
}




void tsp_uart6_receive(void)
{
	uint8_t data;

	if(!DL_UART_isRXFIFOEmpty(K230_INST))
	{
		/* read one byte from the receive data register */
		data = (uint8_t)DL_UART_Main_receiveData(K230_INST);
        if(data == '\n' || data == '\r') {
            // 处理换行符或回车符
            if(rx_idx > 0) {
                rx_buffer[rx_idx] = '\0'; // 结束字符串
                rx_idx = 0; // 重置索引
                rx_flag = 1; // 读到了一个完整的命令
                
            }
        } else if(rx_idx < 255) {
            // 确保不会溢出缓冲区
            // 将数据存入缓冲区
            rx_buffer[rx_idx++] = data;
        }
	}
}

//修改了UART6 的中断处理函数