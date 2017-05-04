#include "stm32f10x.h"

#define	MPU6050_Addr    0xD0 //MPU6050传感器器件地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)


typedef struct
{
	int x;
	int y;
	int z;
}ACCEL; //定义记录三轴加速度的结构体

                   void MPU6050_Init(void);
ACCEL MPU6050_GetValue_ACCEL(void);