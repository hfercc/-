#include <stdio.h>
#include <stm32f10x.h>

#define I2C1_SLAVE_ADDRESS7    0x3A
void I2C(void);
//IIC��ʼ��
void I2C_WriteByte(unsigned char id,unsigned char write_address,unsigned char byte);
//I2Cд����
unsigned char I2C_ReadByte(unsigned char  id, unsigned char read_address);
