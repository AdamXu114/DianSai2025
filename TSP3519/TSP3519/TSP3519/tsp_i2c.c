#include "ti_msp_dl_config.h"
#include "tsp_i2c.h"

#define I2C_TIMEOUT_MS  (10)

int mspm0_i2c_disable(void)
{
    DL_I2C_reset(MPU6050_INST);
    DL_GPIO_initDigitalOutput(GPIO_MPU6050_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_MPU6050_IOMUX_SDA,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_MPU6050_SCL_PORT, GPIO_MPU6050_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_MPU6050_SCL_PORT, GPIO_MPU6050_SCL_PIN);
    return 0;
}

int mspm0_i2c_enable(void)
{
    DL_I2C_reset(MPU6050_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_IOMUX_SDA,
        GPIO_MPU6050_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_MPU6050_IOMUX_SCL,
        GPIO_MPU6050_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_MPU6050_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_MPU6050_IOMUX_SCL);
    DL_I2C_enablePower(MPU6050_INST);
    SYSCFG_DL_MPU6050_init();
    return 0;
}

void mspm0_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable();
    do
    {
        DL_GPIO_clearPins(GPIO_MPU6050_SCL_PORT, GPIO_MPU6050_SCL_PIN);
        delay_1ms(1);
        DL_GPIO_setPins(GPIO_MPU6050_SCL_PORT, GPIO_MPU6050_SCL_PIN);
        delay_1ms(1);

        if(DL_GPIO_readPins(GPIO_MPU6050_SDA_PORT, GPIO_MPU6050_SDA_PIN))
            break;
    }while(++cycleCnt < 100);
    mspm0_i2c_enable();
}

int mspm0_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data)
{
    unsigned int cnt = length;
    unsigned char const *ptr = data;
    unsigned long start, cur;

    if (!length)
        return 0;

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6050_INST, reg_addr);
    DL_I2C_clearInterruptStatus(MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(MPU6050_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_TX, length+1);

    do {
        unsigned fillcnt;
        fillcnt = DL_I2C_fillControllerTXFIFO(MPU6050_INST, ptr, cnt);
        cnt -= fillcnt;
        ptr += fillcnt;

        mspm0_get_clock_ms(&cur);
        if(cur >= (start + I2C_TIMEOUT_MS))
        {
            mspm0_i2c_sda_unlock();
            return -1;
        }
    } while (!DL_I2C_getRawInterruptStatus(MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE));

    return 0;
}

int mspm0_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data)
{
    unsigned i = 0;
    unsigned long start, cur;

    if (!length)
        return 0;

    mspm0_get_clock_ms(&start);

    DL_I2C_transmitControllerData(MPU6050_INST, reg_addr);
    MPU6050_INST->MASTER.MCTR = I2C_MCTR_RD_ON_TXEMPTY_ENABLE;
    DL_I2C_clearInterruptStatus(MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(MPU6050_INST, slave_addr, DL_I2C_CONTROLLER_DIRECTION_RX, length);

    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST))
        {
            uint8_t c;
            c = DL_I2C_receiveControllerData(MPU6050_INST);
            if (i < length)
            {
                data[i] = c;
                ++i;
            }
        }
        
        mspm0_get_clock_ms(&cur);
        if(cur >= (start + I2C_TIMEOUT_MS))
        {
            mspm0_i2c_sda_unlock();
            return -1;
        }
    } while(!DL_I2C_getRawInterruptStatus(MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    if (!DL_I2C_isControllerRXFIFOEmpty(MPU6050_INST))
    {
        uint8_t c;
        c = DL_I2C_receiveControllerData(MPU6050_INST);
        if (i < length)
        {
            data[i] = c;
            ++i;
        }
    }

    MPU6050_INST->MASTER.MCTR = 0;
    DL_I2C_flushControllerTXFIFO(MPU6050_INST);

    if(i == length)
        return 0;
    else
        return -1;
}