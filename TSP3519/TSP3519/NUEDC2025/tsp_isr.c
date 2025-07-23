#include "tsp_isr.h"

volatile uint32_t sys_tick_counter=0;
volatile static uint32_t delay;
extern uint8_t flag_20_ms;
void delay_1ms(uint32_t count)
{
	delay = count;
	while(0U != delay) {}
}

void SysTick_Handler()
{
	sys_tick_counter++;
	if(0U != delay) {
		delay--;
	}
	if (!(sys_tick_counter % 20))
	{
		// LED_R_TOGGLE();
		flag_20_ms = 1;
	}
}
uint32_t get_systick_counter(void)
{
	return sys_tick_counter;
}
int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = sys_tick_counter;
    return 0;
}
/*
void UART0_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART0))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART0, UART0->CPU_INT.RIS);
}

void UART1_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART1))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART1, UART1->CPU_INT.RIS);
}

void UART2_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART2))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART2, UART2->CPU_INT.RIS);
}

void UART3_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART3))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART3, UART3->CPU_INT.RIS);
}
*/


void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);
    if (pending & PORTC_MPU6050_INT_PIN) {
        // —— 你的业务处理 —— 
		// 例如：读取 MPU6050 数据
		short gyro[3];
		MPU6050ReadGyro(gyro);
		int16_t gz = gyro[2];
		// tsp_tft18_show_int16(0, 6, gz);
		tsp_tft18_show_str(0, 6, "Gyro Z:");
        // 一定要清标志，否则会一直回到这里
        DL_GPIO_clearInterruptStatus(PORTC_PORT, PORTC_MPU6050_INT_PIN);
    }
    // 如果 PC4、PC5、PC9 等也开了中断，可继续 if (pending & OTHER_PIN) …
}