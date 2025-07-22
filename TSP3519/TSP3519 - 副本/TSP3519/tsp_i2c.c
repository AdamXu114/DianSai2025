

#include "tsp_i2c.h"
#include "ti_msp_dl_config.h"

// 通用I2C初始化（主机模式，用户可自定义具体实现）
void TSP_I2C_Init(void *i2c_inst)
{
    // 用户可根据实际情况初始化不同I2C实例
    // 例如: SYSCFG_DL_MPU6050_init(); 或自定义
    // 此处仅保留接口，具体实现由用户决定
    SYSCFG_DL_MPU6050_init(); 
}

// 通用I2C单字节写（底层调用多字节写）
uint8_t tsp_i2c_write_byte(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return tsp_i2c_write_bytes(i2c_inst, dev_addr, reg_addr, 1, &data);
}

// 通用I2C单字节读（底层调用多字节读）
uint8_t tsp_i2c_read_byte(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return tsp_i2c_read_bytes(i2c_inst, dev_addr, reg_addr, 1, data);
}

// 通用I2C多字节写（FIFO方式，支持多实例）
int tsp_i2c_write_bytes(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data_ptr)
{
    // 先发寄存器地址
    DL_I2C_fillControllerTXFIFO(i2c_inst, &reg_addr, 1);
    // 再发数据
    DL_I2C_fillControllerTXFIFO(i2c_inst, (uint8_t *)data_ptr, len);
    // 启动传输（TX方向，长度=寄存器+数据）
    DL_I2C_startControllerTransfer(i2c_inst, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, len + 1);
    // 等待总线空闲
    while (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY) {}
    // 错误处理
    if (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_reset(i2c_inst);
        return 1;
    }
    return 0;
}

// 通用I2C多字节读（FIFO方式，支持多实例）
int tsp_i2c_read_bytes(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data_ptr)
{
    // 先写寄存器地址
    DL_I2C_fillControllerTXFIFO(i2c_inst, &reg_addr, 1);
    DL_I2C_startControllerTransfer(i2c_inst, dev_addr , DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY) {}
    if (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_reset(i2c_inst);
        return 1;
    }
    // 再读数据
    DL_I2C_startControllerTransfer(i2c_inst, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    while (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_BUSY) {}
    for (uint16_t i = 0; i < len; i++) {
        data_ptr[i] = DL_I2C_receiveControllerData(i2c_inst);
    }
    if (DL_I2C_getControllerStatus(i2c_inst) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        DL_I2C_reset(i2c_inst);
        return 1;
    }
    return 0;
}

