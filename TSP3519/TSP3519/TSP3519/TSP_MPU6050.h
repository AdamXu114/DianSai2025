#ifndef TSP_MPU6050_H
#define TSP_MPU6050_H
#include "tsp_i2c.h"
#include "ti_msp_dl_config.h"

#define MPU6050_ADDRESS         0xD0  // 8位地址，写操作
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C
 
#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48
 
#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75




int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num);
uint8_t MPU6050ReadID(void);
void MPU6050_Init(void);
void MPU6050ReadAcc(short *accData);
void MPU6050ReadGyro(short *gyroData);


#endif /* TSP_MPU6050_H */
