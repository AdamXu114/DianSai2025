#include "tsp_qei.h"
#include "tsp_common_headfile.h"

// QEI相关变量
static volatile int32_t qei1_last_count = 0;
static volatile int32_t qei2_last_count = 0;
static volatile int8_t qei1_direction = 0;
static volatile int8_t qei2_direction = 0;

// 假定使用 QEI_1_INST 作为编码器硬件实例
// 可根据实际硬件修改


void TSP_QEI_Init(void)
{
    // 初始化 QEI硬件（由SysConfig生成的初始化函数）
    SYSCFG_DL_QEI_1_init();
    SYSCFG_DL_QEI_2_init();
    qei1_last_count = 0;
    qei2_last_count = 0;
    qei1_direction = 0;
    qei2_direction = 0;
}


int32_t TSP_QEI1_GetCount(void)
{
    // 获取 QEI1 的计数值
    // 使用 DL_Timer_getTimerCount 函数获取计数值
    qei1_last_count = DL_Timer_getTimerCount(QEI_1_INST);
    return qei1_last_count;
    
}

int32_t TSP_QEI2_GetCount(void)
{
    qei2_last_count = DL_Timer_getTimerCount(QEI_2_INST);
    return qei2_last_count;
}


void TSP_QEI1_ResetCount(void)
{
    DL_Timer_setCounterValueAfterEnable(QEI_1_INST, 0);
    qei1_last_count = 0;
    qei1_direction = 0;
}

void TSP_QEI2_ResetCount(void)
{
    DL_Timer_setCounterValueAfterEnable(QEI_2_INST, 0);
    qei2_last_count = 0;
    qei2_direction = 0;
}


int8_t TSP_QEI1_GetDirection(void)
{
    int32_t now = DL_Timer_getTimerCount(QEI_1_INST);
    if (now > qei1_last_count)
        qei1_direction = 1;
    else if (now < qei1_last_count)
        qei1_direction = -1;
    else
        qei1_direction = 0;
    qei1_last_count = now;
    return qei1_direction;
}

int8_t TSP_QEI2_GetDirection(void)
{
    int32_t now = DL_Timer_getTimerCount(QEI_2_INST);
    if (now > qei2_last_count)
        qei2_direction = 1;
    else if (now < qei2_last_count)
        qei2_direction = -1;
    else
        qei2_direction = 0;
    qei2_last_count = now;
    return qei2_direction;
}

// 如需中断处理，可在此添加QEI相关中断服务函数
// void QEI_IRQHandler(void) { ... }
