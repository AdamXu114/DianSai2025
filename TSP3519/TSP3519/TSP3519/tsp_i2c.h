#ifndef TSP_I2C_H
#define TSP_I2C_H
#include "tsp_gpio.h"
#include "ti_msp_dl_config.h"

int mspm0_i2c_disable(void);
int mspm0_i2c_enable(void);
void mspm0_i2c_sda_unlock(void);
int mspm0_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);
int mspm0_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

#endif /* TSP_I2C_H */
