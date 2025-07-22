#ifndef TSP_I2C_H
#define TSP_I2C_H
#include "tsp_gpio.h"
#include "ti_msp_dl_config.h"

void TSP_I2C_Init(void *i2c_inst);
uint8_t tsp_i2c_write_byte(void *i2c_inst, uint8_t dev_addr, uint8_t reg, uint8_t data);
uint8_t tsp_i2c_read_byte(void *i2c_inst, uint8_t dev_addr, uint8_t reg, uint8_t *data);
int tsp_i2c_write_bytes(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint16_t len, const uint8_t *data_ptr);
int tsp_i2c_read_bytes(void *i2c_inst, uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data_ptr);
#endif /* TSP_I2C_H */
