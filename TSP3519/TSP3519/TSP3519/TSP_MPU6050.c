#include "TSP_MPU6050.h"
#include "tsp_isr.h"
#include "TSP_TFT18.h"


// 写数据到MPU6050寄存器
int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    return mspm0_i2c_write(MPU6050_ADDRESS >> 1, reg_add, 1, &reg_dat);
}

// 读寄存器：先写寄存器地址，再读一个字节
uint8_t MPU6050_ReadReg(uint8_t reg_addr) {
    uint8_t value = 0;
    mspm0_i2c_read(MPU6050_ADDRESS >> 1, reg_addr, 1, &value);
    return value;
}

// 从MPU6050寄存器读取数据
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num)
{   
    return mspm0_i2c_read(MPU6050_ADDRESS >> 1, reg_add, num, Read);
}

uint8_t MPU6050ReadID(void)
{
   uint8_t Re = MPU6050_ReadReg(MPU6050_WHO_AM_I);        //返回WHO_AM_I寄存器的值
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
    mspm0_i2c_enable();    // I2C硬件初始化
    delay_1ms(100);
    /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);        //电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);        //电源管理寄存器2，保持默认值0，所有轴均不待机
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);        //采样率分频寄存器，配置采样率
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);            //配置寄存器，配置DLPF
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);    //陀螺仪配置寄存器，选择满量程为±2000°/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);    //加速度计配置寄存器，选择满量程为±16g
    delay_1ms(200);
    Gyro_Calibrate(); // 陀螺仪校准
}

float bias[3]={0,0,0};
volatile float z=0;
void MPU6050ReadGyro(short *gyroData)
{
   uint8_t buf[6];
   if (mspm0_i2c_read(MPU6050_ADDRESS >> 1, MPU6050_GYRO_XOUT_H, 6, buf) == 0) {
       gyroData[0] = (buf[0] << 8) | buf[1];
       gyroData[1] = (buf[2] << 8) | buf[3];
       gyroData[2] = (buf[4] << 8) | buf[5];
       //tsp_tft18_show_str(0, 1, "MPU6050 Read OK");
   } else {
       gyroData[0] = 0;
       gyroData[1] = 0;
       gyroData[2] = 0;
       tsp_tft18_show_str(0, 7, "MPU6050 Read Error");
   }
   z = gyroData[2];
}
void Gyro_Calibrate(void) {
    float sum[3]={0,0,0};
    for(int i=0;i<1000;i++){
        short raw[3];
        MPU6050ReadGyro(raw);
        sum[0]+=raw[0];
        sum[1]+=raw[1];
        sum[2]+=raw[2];
        delay_1ms(1);
    }
    bias[0]=sum[0]/1000;
    bias[1]=sum[1]/1000;
    bias[2]=sum[2]/1000;
}
void Gyro_GetAngularRate(float *dps) {
    short raw[3];
    MPU6050ReadGyro(raw);
    // 减去偏置并换算为 °/s
    // dps[0] = (raw[0]-bias[0]) / GYRO_SENS;
    // dps[1] = (raw[1]-bias[1]) / GYRO_SENS;
    // dps[2] = (raw[2]-bias[2]) / GYRO_SENS;
    dps[0] = (raw[0]-bias[0]);
    dps[1] = (raw[1]-bias[1]);
    dps[2] = (raw[2]-bias[2]);
    // char str[30];
    // sprintf(str, "Gyro Z: %6.1f\n", dps[2]);
    // //tsp_tft18_show_str(0, 6, str);
    // tsp_tft18_show_str(0, 5, "NO");
}

float normalize_angle(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}