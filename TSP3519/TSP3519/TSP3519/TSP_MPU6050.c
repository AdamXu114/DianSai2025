#include "TSP_MPU6050.h"
#include "tsp_isr.h"
#include "TSP_TFT18.h"


// 写数据到MPU6050寄存器
// 函数：MPU6050_WriteReg
// 功能：向MPU6050写入一个寄存器
// 参数：reg_add：寄存器地址；reg_dat：要写入的数据
// 返回值：返回写入结果
int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    return mspm0_i2c_write( MPU6050_ADDRESS >> 1, reg_add, 1, &reg_dat);
}

// 函数：MPU6050_ReadReg
// 功能：从MPU6050读取一个寄存器的值
// 参数：reg_add：寄存器地址
// 返回值：返回读取到的数据
uint8_t MPU6050_ReadReg(uint8_t RegAddress) {
    uint8_t value;
    // 内部会用两次传输：写地址，然后重复 START + 读出数据
    mspm0_i2c_read(MPU6050_ADDRESS >> 1,  // 7 位地址
                       RegAddress,
                       1,
                       &value);
    return value;
}

// 从MPU6050寄存器读取数据
// 函数：MPU6050_ReadData
// 功能：从MPU6050读取多个寄存器的值
// 参数：reg_add：寄存器地址；Read：读取的数据；num：读取的寄存器个数
// 返回值：返回读取结果
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num)
{
    return mspm0_i2c_read(MPU6050_ADDRESS >> 1, reg_add, num, Read);
}

// 函数：MPU6050ReadID
// 功能：读取MPU6050的ID
// 参数：无
// 如果读取失败，则返回0
uint8_t MPU6050ReadID(void)
{
   uint8_t Re = MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
   if (Re != 0x68) {
         tsp_tft18_show_str(0,7,"MPU6050 Not Found");
         return 0;
   } else {
        char str[30];
        sprintf(str, "MPU6050 ID = %d", Re);
        tsp_tft18_show_str(0,7, str);
        return 1;
   }
}

// 函数：MPU6050_Init
// 功能：初始化MPU6050
// 参数：无
// 返回值：无
void MPU6050_Init(void)
{
    if (!MPU6050ReadID()) {
        return ; // 如果没有找到 MPU6050，直接返回
    }
    //mspm0_i2c_enable();    // 硬件 I2C 一次性配置
    delay_1ms(100);
/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
    delay_1ms(200);
}

void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACCEL_XOUT_H, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
    tsp_tft18_show_int16(0, 7, accData[0]); // X轴
    tsp_tft18_show_int16(40, 7, accData[1]); // Y轴
    tsp_tft18_show_int16(80, 7, accData[2]); // Z轴
}


// 函数：MPU6050ReadGyro
// 功能：读取MPU6050的陀螺仪数据
// 参数：gyroData：陀螺仪数据
// 返回值：无
void MPU6050ReadGyro(short *gyroData)
{
   uint8_t buf[6];
   if (!MPU6050_ReadData(MPU6050_GYRO_XOUT_H, buf, 6)){
       gyroData[0] = (buf[0] << 8) | buf[1];
       gyroData[1] = (buf[2] << 8) | buf[3];
       gyroData[2] = (buf[4] << 8) | buf[5];
   }
   else {
       gyroData[0] = 99;
       gyroData[1] = 99;
       gyroData[2] = 99;
   }
    tsp_tft18_show_int16(0, 6, gyroData[0]); // X轴
    tsp_tft18_show_int16(40, 6, gyroData[1]); // Y轴
    tsp_tft18_show_int16(80, 6, gyroData[2]); // Z轴
}

