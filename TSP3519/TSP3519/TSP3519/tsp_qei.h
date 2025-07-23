#ifndef TSP_QEI_H
#define TSP_QEI_H
#include "tsp_gpio.h"
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// QEI 编码器相关接口声明

/**
 * @brief 初始化 QEI 编码器
 */
void TSP_QEI_Init(void);

/**
 * @brief 获取编码器计数值
 * @return 当前计数值
 */
int32_t TSP_QEI1_GetCount(void);
int32_t TSP_QEI2_GetCount(void);
/**
 * @brief 复位编码器计数值
 */
void TSP_QEI1_ResetCount(void);
void TSP_QEI2_ResetCount(void);
/**
 * @brief 获取编码器方向
 * @return 1: 正向, -1: 反向, 0: 未知
 */
int8_t TSP_QEI1_GetDirection(void);
int8_t TSP_QEI2_GetDirection(void);






#ifdef __cplusplus
}
#endif

#endif /* TSP_QEI_H */