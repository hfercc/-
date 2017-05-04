
#include "stm32f10x.h"
#include "math.h"
#include "MPU6050.h"
#include	"I2C.h"


void MPU6050_Init() //对MPU6050进行初始化配置
{
   
	I2C();//使用STM32的硬件I2C，先对其进行初始化配置
	I2C_WriteByte(MPU6050_Addr,PWR_MGMT_1, 0x00);//设置电源管理寄存器值，典型值：0x00(正常启用)
	I2C_WriteByte(MPU6050_Addr,CONFIG, 0x06);   //陀螺仪和加速度计低通滤波频率，典型值：0x06(5Hz)
	I2C_WriteByte(MPU6050_Addr,ACCEL_CONFIG, 0x01);//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2g，5Hz)
}

ACCEL MPU6050_GetValue_ACCEL(void) //读取三轴加速度
{
	ACCEL AccelValue;
	
	AccelValue.x = (u16)I2C_ReadByte(MPU6050_Addr,ACCEL_XOUT_H); //读取x轴上加速度中的高8位，并将其强制转换为16位

	AccelValue.x <<= 8;																					//左移8位，将高8位转换到16位中的高位
	AccelValue.x += (u16)I2C_ReadByte(MPU6050_Addr,ACCEL_XOUT_L); //读取x轴加速度中的低8位，并将其与高8位合并，得到16位补码的X轴加速度

	
	if(AccelValue.x > 0x8000)						//将无符号的16位补码转换为int
	{
		AccelValue.x = AccelValue.x-0xFFFF;
		
	}

  return AccelValue;
}
