#include "HSP_MMA8451.h"
#include <stdio.h>

// https://community.nxp.com/t5/Sensors-Knowledge-Base/MMA8451Q-Bare-metal-example-project/ta-p/1127268

void hsp_mma8451_config()
{
    uint8_t write_address;
    uint8_t buffer_write[2];
	uint8_t temp1, reg_val;

	do
	{
		hsp_mma8451_read_byte(WHO_AM_I_REG, &temp1);
		
	} while(temp1 != MMA8451Q_ID);
	printf("\nMMA8451Q detected\n\r");

//	hsp_mma8451_read_byte(CTRL_REG1, &temp1);
//	hsp_mma8451_write_byte(CTRL_REG2, temp1|0x01);
	hsp_mma8451_write_byte(CTRL_REG1, 0x05);
	hsp_mma8451_write_byte(CTRL_REG2, 0x02);
	hsp_mma8451_write_byte(XYZ_DATA_CFG_REG, 0x10);
}

void hsp_i2c_mma8451_init()
{
	// enable peripheral clock
	rcu_periph_clock_enable(RCU_GPIOF);
	rcu_periph_clock_enable(RCU_I2C1);

	// PF0/I2C1_SDA, PF1/I2C1_SCL
	gpio_af_set(GPIOF, GPIO_AF_4, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_output_options_set(GPIOF, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);

	// configure I2C clock
	//i2c_clock_config(MMA8451_I2C_BASEADDR, MMA8451_I2C_SPEED, I2C_DTCY_2);
	i2c_clock_config(MMA8451_I2C_BASEADDR, MMA8451_I2C_SPEED, I2C_DTCY_16_9);
	// configure I2C address
	i2c_mode_addr_config(MMA8451_I2C_BASEADDR, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, MMA8451_I2C_ADDR7);
	// enable I2C1
	i2c_enable(MMA8451_I2C_BASEADDR);
	// enable acknowledge
	i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_ENABLE);
	//i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_DISABLE);
	i2c_analog_noise_filter_enable(MMA8451_I2C_BASEADDR);

	hsp_mma8451_config();
}

uint8_t hsp_mma8451_read_byte(uint8_t reg, uint8_t *data)
{
	volatile uint32_t timeout = MMA8451_I2C_FLAG_TIMEOUT;

	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	/* generate the start condition */
	i2c_start_on_bus(MMA8451_I2C_BASEADDR);

	/* test on I2C I2C_SBSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_SBSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 
	/* send DCI selected device slave address for read */
	i2c_master_addressing(MMA8451_I2C_BASEADDR, MMA8451_I2C_WRITE_ADDRESS, I2C_TRANSMITTER);

	/* test on I2C I2C_ADDSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	}
	i2c_flag_clear(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND);

	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_TBE)){
		if ((timeout--) == 0) 
			return 0xFF;
	}

	/* send register address */
	i2c_data_transmit(MMA8451_I2C_BASEADDR, (uint8_t)(reg));

	/* test on I2C I2C_BTC and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_BTC)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;
	  
	/* generate the start condition */
	i2c_start_on_bus(MMA8451_I2C_BASEADDR);

	/* test on I2C I2C_SBSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_SBSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* send DCI selcted device slave address for write */
	i2c_master_addressing(MMA8451_I2C_BASEADDR, MMA8451_I2C_READ_ADDRESS, I2C_RECEIVER);

	/* test on I2C I2C_ADDSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	}
	i2c_flag_clear(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND); 

	/* prepare an nack for the next data received */
	i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_DISABLE);

	/* test on I2C_RBNE and clear it */
	timeout = HSP_SCCB_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_RBNE)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* receive the data */
	*data = i2c_data_receive(MMA8451_I2C_BASEADDR);

	/* prepare stop after receiving data */
	i2c_stop_on_bus(MMA8451_I2C_BASEADDR); 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	return 0;
}

uint8_t hsp_mma8451_read_nbyte(uint8_t reg, uint8_t *data_addr, uint8_t n)
{
	volatile uint32_t timeout = MMA8451_I2C_FLAG_TIMEOUT;
	uint8_t num;

	num = n;

	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	/* generate the start condition */
	i2c_start_on_bus(MMA8451_I2C_BASEADDR);

	/* test on I2C I2C_SBSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_SBSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 
	/* send selected device slave address for read */
	i2c_master_addressing(MMA8451_I2C_BASEADDR, MMA8451_I2C_WRITE_ADDRESS, I2C_TRANSMITTER);

	/* test on I2C I2C_ADDSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	}
	i2c_flag_clear(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND);

	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_TBE)){
		if ((timeout--) == 0) 
			return 0xFF;
	}

	/* send register address */
	i2c_data_transmit(MMA8451_I2C_BASEADDR, (uint8_t)(reg));

	/* test on I2C I2C_BTC and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_BTC)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;
	  
	/* prepare stop after receiving data */
	//i2c_stop_on_bus(MMA8451_I2C_BASEADDR);

	/* generate the start condition */
	i2c_start_on_bus(MMA8451_I2C_BASEADDR);

	/* test on I2C I2C_SBSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_SBSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* send selcted device slave address for write */
	i2c_master_addressing(MMA8451_I2C_BASEADDR, MMA8451_I2C_READ_ADDRESS, I2C_RECEIVER);

	/* test on I2C I2C_ADDSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	}
	i2c_flag_clear(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND); 

	//i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_ENABLE);
	while (--num)
	{
		/* prepare an ack for the next data received */
		i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_ENABLE);

		/* test on I2C_RBNE and clear it */
		timeout = HSP_SCCB_FLAG_TIMEOUT;
		while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_RBNE)){
			if ((timeout--) == 0) 
				return 0xFF;
		} 

		/* receive the data */
		*data_addr = i2c_data_receive(MMA8451_I2C_BASEADDR);

		/* clear AF flag if arised */
		I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

		data_addr++;
	}

	/* prepare an nack for the next data received */
    i2c_ack_config(MMA8451_I2C_BASEADDR, I2C_ACK_DISABLE);

	/* test on I2C_RBNE and clear it */
	timeout = HSP_SCCB_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_RBNE)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* receive the data */
	*data_addr = i2c_data_receive(MMA8451_I2C_BASEADDR);

	/* prepare stop after receiving data */
	i2c_stop_on_bus(MMA8451_I2C_BASEADDR); 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	return 0;
}

uint8_t hsp_mma8451_write_byte(uint8_t reg, uint8_t data)
{
	volatile uint32_t timeout = MMA8451_I2C_FLAG_TIMEOUT;

	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	/* generate the start condition */
	i2c_start_on_bus(MMA8451_I2C_BASEADDR);

	/* test on I2C I2C_SBSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT;
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_SBSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 
	/* send selected device slave address for write */
	i2c_master_addressing(MMA8451_I2C_BASEADDR, MMA8451_I2C_WRITE_ADDRESS, I2C_TRANSMITTER);

	/* test on I2C I2C_ADDSEND and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND)){
		if ((timeout--) == 0) 
			return 0xFF;
	}
	i2c_flag_clear(MMA8451_I2C_BASEADDR, I2C_FLAG_ADDSEND);

	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_TBE)){
		if ((timeout--) == 0) 
			return 0xFF;
	}

	/* send register address */
	i2c_data_transmit(MMA8451_I2C_BASEADDR, (uint8_t)(reg));

	/* test on I2C I2C_BTC and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_BTC)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;
	  
	/* send register address */
	i2c_data_transmit(MMA8451_I2C_BASEADDR, data);

	/* test on I2C I2C_BTC and clear it */
	timeout = MMA8451_I2C_FLAG_TIMEOUT; 
	while(!i2c_flag_get(MMA8451_I2C_BASEADDR, I2C_FLAG_BTC)){
		if ((timeout--) == 0) 
			return 0xFF;
	} 

	/* prepare stop after receiving data */
	i2c_stop_on_bus(MMA8451_I2C_BASEADDR); 

	/* clear AF flag if arised */
	I2C_STAT0(MMA8451_I2C_BASEADDR) |= (uint16_t)0x0400;

	return 0;
}

// 软件 I2C 时序延时（大约几微秒，根据系统时钟微调）
static void soft_i2c_delay(void) {
    for (volatile int i = 0; i < 50; i++);
}

// PE2 -> SDA，PE3 -> SCL
#define SOFT_I2C_SDA_H()    gpio_bit_set(GPIOE, GPIO_PIN_2)
#define SOFT_I2C_SDA_L()    gpio_bit_reset(GPIOE, GPIO_PIN_2)
#define SOFT_I2C_SCL_H()    gpio_bit_set(GPIOE, GPIO_PIN_3)
#define SOFT_I2C_SCL_L()    gpio_bit_reset(GPIOE, GPIO_PIN_3)
#define SOFT_I2C_SDA_READ() gpio_input_bit_get(GPIOE, GPIO_PIN_2)

// 软件 I2C 初始化
void hsp_soft_i2c_init(void)
{
    // 1. 使能 GPIOE 时钟
    rcu_periph_clock_enable(RCU_GPIOE);
    // 2. 配置 PE2/PE3 为开漏输出，上拉
    gpio_mode_set(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_2 | GPIO_PIN_3);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_2 | GPIO_PIN_3);
    // 3. 先拉高空闲
    SOFT_I2C_SDA_H();
    SOFT_I2C_SCL_H();
}

// I²C 起始信号：SDA 由高拉低时 SCL 高
void soft_i2c_start(void)
{
    SOFT_I2C_SDA_H();  SOFT_I2C_SCL_H();  soft_i2c_delay();
    SOFT_I2C_SDA_L();  soft_i2c_delay();
    SOFT_I2C_SCL_L();  soft_i2c_delay();
}

// I²C 停止信号：SDA 在 SCL 高期间由低拉高
void soft_i2c_stop(void)
{
    SOFT_I2C_SDA_L();  soft_i2c_delay();
    SOFT_I2C_SCL_H();  soft_i2c_delay();
    SOFT_I2C_SDA_H();  soft_i2c_delay();
}

// 发送 1 位（先写数据线，再拉高时钟）
void soft_i2c_write_bit(uint8_t bit)
{
    if (bit) SOFT_I2C_SDA_H();
    else     SOFT_I2C_SDA_L();
    soft_i2c_delay();
    SOFT_I2C_SCL_H();
    soft_i2c_delay();
    SOFT_I2C_SCL_L();
    soft_i2c_delay();
}

// 读 1 位：释放 SDA（上拉），在时钟高期间读值
uint8_t soft_i2c_read_bit(void)
{
    uint8_t bit;
    SOFT_I2C_SDA_H();  // 释放 SDA，线被上拉
    soft_i2c_delay();
    SOFT_I2C_SCL_H();
    soft_i2c_delay();
    bit = SOFT_I2C_SDA_READ();
    SOFT_I2C_SCL_L();
    soft_i2c_delay();
    return bit;
}

// 发送一个字节，返回 ACK（0 表示从机应答）
uint8_t soft_i2c_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        soft_i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    // 读 ACK
    return soft_i2c_read_bit();
}

// 从机读一个字节，master 发 ACK=1/NACK=0
uint8_t soft_i2c_read_byte(uint8_t ack)
{
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte <<= 1;
        byte |= soft_i2c_read_bit();
    }
    // 发送 ACK/NACK
    soft_i2c_write_bit(!ack);
    return byte;
}

// MPU6050 写寄存器
// void mpu6050_write_reg(uint8_t reg, uint8_t data)
// {
//     soft_i2c_start();
//     soft_i2c_write_byte(0xD0);           // MPU6050 I2C 地址 <<1 + 写
//     soft_i2c_write_byte(reg);
//     soft_i2c_write_byte(data);
//     soft_i2c_stop();
// }
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len,
                              unsigned char *data_ptr)
{
    soft_i2c_start();
    if (soft_i2c_write_byte(slave_addr & 0xFE)) { // 写操作，最低位为0
        soft_i2c_stop();
        return -1;
    }
    if (soft_i2c_write_byte(reg_addr)) {
        soft_i2c_stop();
        return -2;
    }
    for (unsigned short i = 0; i < len; i++) {
        if (soft_i2c_write_byte(data_ptr[i])) {
            soft_i2c_stop();
            return -3;
        }
    }
    soft_i2c_stop();
    return 0;
}

// MPU6050 读寄存器
// uint8_t mpu6050_read_reg(uint8_t reg)
// {
//     uint8_t val;
//     soft_i2c_start();
//     soft_i2c_write_byte(0xD0);           // 写寄存器地址
//     soft_i2c_write_byte(reg);
//     soft_i2c_start();
//     soft_i2c_write_byte(0xD1);           // 读取模式
//     val = soft_i2c_read_byte(0);         // 最后一个字节 NACK
//     soft_i2c_stop();
//     return val;
// }
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len,
                             unsigned char *data_ptr)
{
    soft_i2c_start();
    if (soft_i2c_write_byte(slave_addr & 0xFE)) { // 写操作，最低位为0
        soft_i2c_stop();
        return -1;
    }
    if (soft_i2c_write_byte(reg_addr)) {
        soft_i2c_stop();
        return -2;
    }
    soft_i2c_start();
    if (soft_i2c_write_byte(slave_addr | 0x01)) { // 读操作，最低位为1
        soft_i2c_stop();
        return -3;
    }
    for (unsigned short i = 0; i < len; i++) {
        data_ptr[i] = soft_i2c_read_byte(i == (len - 1) ? 0 : 1); // 最后一个字节NACK
    }
    soft_i2c_stop();
    return 0;
}

void MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    Sensors_I2C_WriteRegister(MPU6050_ADDRESS, reg_add, 1, &reg_dat);
}

void MPU6050_ReadData(uint8_t reg_add, unsigned char* Read, uint8_t num)
{
    Sensors_I2C_ReadRegister(MPU6050_ADDRESS, reg_add, num, Read);
}

void MPU6050_Init(void)
{
    soft_i2c_delay();
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);      // 解除休眠
    MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV, 0x07);      // 采样率
    MPU6050_WriteReg(MPU6050_RA_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG, 0x01);    // 加速度16G
    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     // 陀螺仪2000deg/s
    soft_i2c_delay();
}

// 最终的初始化调用
void hsp_mpu6050_init_on_PE2_PE3(void)
{
    hsp_soft_i2c_init();
    MPU6050_Init();
}


#define MPU6050_RA_TEMP_OUT_H  0x41
#define MPU6050_RA_TEMP_OUT_L  0x42


/**
* @brief   读取MPU6050的加速度原始数据
* @param   accData: short[3]，存储X/Y/Z轴加速度
* @retval  无
*/
void MPU6050_ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_RA_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}

/**
* @brief   读取MPU6050的陀螺仪原始数据
* @param   gyroData: short[3]，存储X/Y/Z轴角速度
* @retval  无
*/
void MPU6050_ReadGyro(short *gyroData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_RA_GYRO_OUT, buf, 6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

/**
* @brief   读取MPU6050的原始温度数据
* @param   tempData: short*，存储温度原始值
* @retval  无
*/
void MPU6050_ReadTempRaw(short *tempData)
{
    uint8_t buf[2];
    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H, buf, 2);
    *tempData = (buf[0] << 8) | buf[1];
}

/**
* @brief   读取MPU6050的温度数据（摄氏度）
* @param   Temperature: float*，存储转换后的温度
* @retval  无
*/
void MPU6050_ReadTempCelsius(float *Temperature)
{
    short temp_raw;
    MPU6050_ReadTempRaw(&temp_raw);
    *Temperature = ((float)temp_raw) / 340.0f + 36.53f;
}