#include "TSP_MPU6050.h"
#include "tsp_isr.h"
#include "TSP_TFT18.h"


// 写数据到MPU6050寄存器
int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    return tsp_i2c_write_bytes(MPU6050_INST, MPU6050_ADDRESS >> 1, reg_add, 1, &reg_dat);
}
// 读寄存器：先写寄存器地址，再读一个字节
uint8_t MPU6050_ReadReg(uint8_t RegAddress) {
    uint8_t value;
    // 内部会用两次传输：写地址，然后重复 START + 读出数据
    tsp_i2c_read_bytes(MPU6050_INST,
                       MPU6050_ADDRESS >> 1,  // 7 位地址
                       RegAddress,
                       1,
                       &value);
    return value;
}

// 从MPU6050寄存器读取数据
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num)
{
    return tsp_i2c_read_bytes(MPU6050_INST, MPU6050_ADDRESS>>1, reg_add, num, Read);
}

uint8_t MPU6050ReadID(void)
{
   uint8_t Re = MPU6050_ReadReg(MPU6050_WHO_AM_I>>1);		//返回WHO_AM_I寄存器的值
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
void MPU6050_Init(void)
{
//    //在初始化之前要延时一段时间，若没有延时，则断电后再上电数据可能会出错

//    //解除休眠状态
//    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);
//    //陀螺仪采样率
//    MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);
//    MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);
//    //配置加速度传感器工作在16G模式
//    MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);
//    //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
//    MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);
    TSP_I2C_Init(MPU6050_INST);    // 硬件 I2C 一次性配置
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
/**
* @brief   读取MPU6050的加速度数据
* @param
* @retval
*/
// void MPU6050ReadAcc(short *accData)
// {
//    uint8_t buf[6];
//    MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
//    accData[0] = (buf[0] << 8) | buf[1];
//    accData[1] = (buf[2] << 8) | buf[3];
//    accData[2] = (buf[4] << 8) | buf[5];
// }

/**
* @brief   读取MPU6050的角加速度数据
* @param
* @retval
*/
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
    tsp_tft18_show_int16(0, 6, gyroData[2]);
}

/**
* @brief   读取MPU6050的原始温度数据
* @param
* @retval
*/
// void MPU6050ReadTemp(short *tempData)
// {
//    uint8_t buf[2];
//    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
//    *tempData = (buf[0] << 8) | buf[1];
// }

/**
* @brief   读取MPU6050的温度数据，转化成摄氏度
* @param
* @retval
*/
// void MPU6050_ReturnTemp(float*Temperature)
// {
//    short temp3;
//    uint8_t buf[2];

//    MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //读取温度值
//    temp3= (buf[0] << 8) | buf[1];
//    *Temperature=((double) (temp3 /340.0))+36.53;
// }