#include "stm32f10x.h"
#include	"I2C.h"


void I2C()
{
	 GPIO_InitTypeDef GPIO_InitStructure;

	 I2C_InitTypeDef I2C_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);  //开启GPIOB和AFIO复用时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);											//开启I2C1的时钟
	/* Configure IO connected to IIC*********************/
	 GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;                	//对I2C1对应的IO口进行初始化
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;                          //注意设为复用开漏输出
 	 GPIO_Init(GPIOB, &GPIO_InitStructure);

	 I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;																//设置模式为I2C模式
 	 I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;                       //设置I2C占空比
 	 I2C_InitStructure.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;									//设置本机I2C的地址
 	 I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;															//开启I2C的应答功能
 	 I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //设置I2C应答地址为7位
 	 I2C_InitStructure.I2C_ClockSpeed = 20000;                                  //设置I2C工作时钟频率
	 
	 I2C_Init(I2C1, &I2C_InitStructure);																					//对I2C1进行初始化
	 I2C_Cmd(I2C1, ENABLE);   																									//使I2C1
}
/**********************************************************************/
/*IIC写一个字节	         				                              */
/*																	  */
/**********************************************************************/
void I2C_WriteByte(unsigned char id,unsigned char write_address,unsigned char byte)
{	
	I2C_GenerateSTART(I2C1,ENABLE);
	//产生起始条件
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	//等待ACK，检查主机是否获得I2C总线
	I2C_Send7bitAddress(I2C1,id,I2C_Direction_Transmitter);
	//向I2C总线上的从设备发送准备通信的设备地址，并表示要写数据
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	//等待ACK，等待是否有该地址的I2C设备回应
	I2C_SendData(I2C1, write_address);
	//寄存器地址，若回应，发送准备写入数据的寄存器的地址
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//等待ACK，等待从设备回应
	I2C_SendData(I2C1, byte);
	//发送数据，发送准备写入寄存器的数据
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	//发送完成，等待从设备回应，表明发送成功
	I2C_GenerateSTOP(I2C1, ENABLE);
	//产生结束信号
}
/**********************************************************************/
/*IIC读数据	         				                              	  */
/*																	  */
/**********************************************************************/
unsigned char I2C_ReadByte(unsigned char  id, unsigned char read_address)
{
	unsigned char temp; 	
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
  	//等待I2C，等待I2C总线回到空闲
  	I2C_GenerateSTART(I2C1, ENABLE);
  	//产生起始信号
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    //EV5，等待ACK
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Transmitter);
	//发送准备通信的从设备地址，并表示要写数据
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
  	//EV6，等待ACK
  	I2C_Cmd(I2C1, ENABLE);
 	//重新设置可以清除EV6
  	I2C_SendData(I2C1, read_address);  
	//发送准备读取的寄存器的地址
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	//EV8 ，等待ACK
  	I2C_GenerateSTART(I2C1, ENABLE);
	//重新产生起始信号，准备读数据
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  	//EV5
  	I2C_Send7bitAddress(I2C1, id, I2C_Direction_Receiver);
  	//发送准备读取的设备的地址
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  	//EV6  
    I2C_AcknowledgeConfig(I2C1, DISABLE);
    
	//关闭应答和停止条件产生
    while(!(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)));
		//等待接收到数据
	      
    temp = I2C_ReceiveData(I2C1);
		//读取接受到的数据
		
		I2C_GenerateSTOP(I2C1, ENABLE);
		//产生结束条件
   
  	I2C_AcknowledgeConfig(I2C1, ENABLE);
		//重新开启应答

	return temp;
}
