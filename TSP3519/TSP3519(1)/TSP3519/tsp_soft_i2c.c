#include "ti_msp_dl_config.h"
#include "tsp_soft_i2c.h"
#include "ti/driverlib/dl_gpio.h"
//打开SDA引脚（输出）
void SDA_OUT(void)   
{
    DL_GPIO_initDigitalOutput(PORTB_GPIO_SDA_IOMUX);     
	DL_GPIO_setPins(GPIOC, PORTB_GPIO_SDA_PIN);	   
    DL_GPIO_enableOutput(GPIOC, PORTB_GPIO_SDA_PIN); 
}
//关闭SDA引脚（输入）
void SDA_IN(void)
{
 
    DL_GPIO_initDigitalInputFeatures(PORTB_GPIO_SDA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
 
 
}
 
void Delay_us(uint16_t us)
{
    while(us--)
    delay_cycles(CPUCLK_FREQ/1000000);
}//CPUCLK_FREQ为时钟频率，可以根据配置的改变而改变
/*引脚配置层*/
 
/**
  * 函    数：I2C写SCL引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SCL的电平，范围0~1
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SCL为低电平，当BitValue为1时，需要置SCL为高电平
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
    if(BitValue)
        DL_GPIO_setPins(GPIOC, PORTB_GPIO_SCL_PIN);
    else
        DL_GPIO_clearPins(GPIOC, PORTB_GPIO_SCL_PIN);
	Delay_us(8);	//延时8us，防止时序频率超过要求
}
 
/**
  * 函    数：I2C写SDA引脚电平
  * 参    数：BitValue 协议层传入的当前需要写入SDA的电平，范围0~0xFF
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，当BitValue为0时，需要置SDA为低电平，当BitValue非0时，需要置SDA为高电平
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
    SDA_OUT();
    if(BitValue)
        DL_GPIO_setPins(GPIOC, PORTB_GPIO_SDA_PIN);
    else
        DL_GPIO_clearPins(GPIOC, PORTB_GPIO_SDA_PIN);
	Delay_us(8);					//延时8us，防止时序频率超过要求
}
 
/**
  * 函    数：I2C读SDA引脚电平
  * 参    数：无
  * 返 回 值：协议层需要得到的当前SDA的电平，范围0~1
  * 注意事项：此函数需要用户实现内容，当前SDA为低电平时，返回0，当前SDA为高电平时，返回1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t b;
    uint32_t BitValue;
    SDA_IN();
	BitValue = DL_GPIO_readPins(GPIOC, PORTB_GPIO_SDA_PIN);		//读取SDA电平
    {
        if(BitValue)   b=1;
        else           b=0;
    }
	Delay_us(8);		//延时8us，防止时序频率超过要求
	return b;	        //返回SDA电平
}
 
/**
  * 函    数：I2C初始化
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数需要用户实现内容，实现SCL和SDA引脚的初始化
  */
void MyI2C_Init(void)
{
    //SYSCFG_DL_GPIO_init();
	/*设置默认电平*/
	DL_GPIO_setPins(GPIOA, PORTB_GPIO_SDA_PIN |
		PORTB_GPIO_SCL_PIN);//设置PA8和PA9引脚初始化后默认为高电平（释放总线状态）
}
 
/*协议层*/
 
/**
  * 函    数：I2C起始
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Start(void)
{
    SDA_OUT();
	MyI2C_W_SDA(1);				//释放SDA，确保SDA为高电平
	MyI2C_W_SCL(1);				//释放SCL，确保SCL为高电平
	MyI2C_W_SDA(0);				//在SCL高电平期间，拉低SDA，产生起始信号
	MyI2C_W_SCL(0);				//起始后拉低SCL，为了占用总线，方便总线时序的拼接
}
 
/**
  * 函    数：I2C终止
  * 参    数：无
  * 返 回 值：无
  */
void MyI2C_Stop(void)
{
    SDA_OUT();
	MyI2C_W_SDA(0);							//拉低SDA，确保SDA为低电平
	MyI2C_W_SCL(1);							//释放SCL，使SCL呈现高电平
	MyI2C_W_SDA(1);							//在SCL高电平期间，释放SDA，产生终止信号
}
 
/**
  * 函    数：I2C发送一个字节
  * 参    数：Byte 要发送的一个字节数据，范围：0x00~0xFF
  * 返 回 值：无
  */
void MyI2C_SendByte(uint8_t Byte)
{
    SDA_OUT();
	uint8_t i;
	for (i = 0; i < 8; i ++)				//循环8次，主机依次发送数据的每一位
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));	//使用掩码的方式取出Byte的指定一位数据并写入到SDA线
		MyI2C_W_SCL(1);						//释放SCL，从机在SCL高电平期间读取SDA
		MyI2C_W_SCL(0);						//拉低SCL，主机开始发送下一位数据
	}
}
 
/**
  * 函    数：I2C接收一个字节
  * 参    数：无
  * 返 回 值：接收到的一个字节数据，范围：0x00~0xFF
  */
uint8_t MyI2C_ReceiveByte(void)
{
    SDA_OUT();
	uint8_t i, Byte = 0x00;	//定义接收的数据，并赋初值0x00
	MyI2C_W_SDA(1);			//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	for (i = 0; i < 8; i ++)	//循环8次，主机依次接收数据的每一位
	{
        SDA_IN();
		MyI2C_W_SCL(1);						//释放SCL，主机机在SCL高电平期间读取SDA
		if (MyI2C_R_SDA() == 1){Byte |= (0x80 >> i);}	//读取SDA数据，并存储到Byte变量
	//当SDA为1时，置变量指定位为1，当SDA为0时，不做处理，指定位为默认的初值0
		MyI2C_W_SCL(0);						//拉低SCL，从机在SCL低电平期间写入SDA
	}
	return Byte;							//返回接收到的一个字节数据
}
 
/**
  * 函    数：I2C发送应答位
  * 参    数：Byte 要发送的应答位，范围：0~1，0表示应答，1表示非应答
  * 返 回 值：无
  */
void MyI2C_SendAck(uint8_t AckBit)
{
    SDA_OUT();
	MyI2C_W_SDA(AckBit);					//主机把应答位数据放到SDA线
	MyI2C_W_SCL(1);							//释放SCL，从机在SCL高电平期间，读取应答位
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
}
 
/**
  * 函    数：I2C接收应答位
  * 参    数：无
  * 返 回 值：接收到的应答位，范围：0~1，0表示应答，1表示非应答
  */
uint8_t MyI2C_ReceiveAck(void)
{
    SDA_OUT();
	uint8_t AckBit;							//定义应答位变量
	MyI2C_W_SDA(1);							//接收前，主机先确保释放SDA，避免干扰从机的数据发送
	MyI2C_W_SCL(1);							//释放SCL，主机机在SCL高电平期间读取SDA
    SDA_IN();
	AckBit = MyI2C_R_SDA();					//将应答位存储到变量里
	MyI2C_W_SCL(0);							//拉低SCL，开始下一个时序模块
	return AckBit;							//返回定义应答位变量
}