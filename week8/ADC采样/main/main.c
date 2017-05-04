
#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"
#include "math.h"

int num[4][10] = {{0x01C0,0x01F9,0x01A4,0x01B0,0x0199,0x0192,0x0182,0x01F8,0x0180,0x0190},
                 {0x0240,0x0279,0x0224,0x0230,0x0219,0x0212,0x0202,0x0278,0x0200,0x0210},
                 {0x04C0,0x04F9,0x04A4,0x04B0,0x0499,0x0492,0x0482,0x04F8,0x0480,0x0490},
                 {0x08C0,0x08F9,0x08A4,0x08B0,0x0899,0x0892,0x0882,0x08F8,0x0880,0x0890}};
#define PI 3.141592
#define SCL_H GPIOA->BSRR=GPIO_Pin_6
#define SCL_L GPIOA->BRR=GPIO_Pin_6
#define SDA_H GPIOA->BSRR=GPIO_Pin_7
#define SDA_L GPIOA->BRR=GPIO_Pin_7
#define SDA_R GPIOA->IDR&GPIO_Pin_7

#define	MPU6050_Addr    0xD0 //MPU6050������������ַ
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	PWR_MGMT_1		0x6B	//��Դ����������ֵ��0x00(��������)

u16 value;

int counter,flag;

void tim3(void);
void tim4(void);

void nvic(void);

int r[4] = {0,0,0,0};

void change(float T) {
    //r[3] = T/1000;
    r[2] = (T-r[3]*1000)/100;
    r[1] = (T-r[2]*100-r[3]*1000)/10;
    r[0] = ((T-r[1]*10-r[2]*100-r[3]*1000));
    if(r[0]<0) r[0]=-r[0];
    if(r[1]<0) r[1]=-r[1];
    if(r[2]<0) r[2]=-r[2];
    if(r[3]<0) r[3]=-r[3];
}
void display(int n[3],int sgn) {
    int i;
    for(i=0;i<3;i++) {
        GPIO_Write(GPIOC,num[i][n[i]]);
    }
    if(sgn>0) {GPIO_Write(GPIOC,0x08FF);}
   else GPIO_Write(GPIOC,0x08BF);
}

void RCC_Configuration(void);	 //ʱ�ӳ�ʼ������������ʱ��
void GPIO_Configuration(void);	 //IO�ڳ�ʼ���������书��

void TIM3_IRQHandler(void)    //    //TIM3�����������ж���Ӧ����
{
      TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //   //  ����TIM3�����ж���Ӧ������־λ
 //��ʼ����ģʽ����ת��
      counter++;
      if(counter>300) counter = 0;
      if(flag>0) {
          if(counter < (flag)/100) {
              GPIO_SetBits(GPIOA,GPIO_Pin_0);
              GPIO_ResetBits(GPIOA,GPIO_Pin_1);
          }
          else {
              GPIO_ResetBits(GPIOA,GPIO_Pin_0);
              GPIO_ResetBits(GPIOA,GPIO_Pin_1);
          }
      }
      else{
          if(counter < (-(flag)/100)) {
              GPIO_SetBits(GPIOA,GPIO_Pin_1);
              GPIO_ResetBits(GPIOA,GPIO_Pin_0);
          }
          else {
              GPIO_ResetBits(GPIOA,GPIO_Pin_1);
              GPIO_ResetBits(GPIOA,GPIO_Pin_0);
          }
      }
}




void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

void delay(void) {
  u8 i = 10;
  while(i) {i--;}
}

u8 I2C_Start(void) {
  SDA_H;
  SCL_H;
  delay();
  if(!SDA_R) return 0;
  SDA_L;
  delay();
  if(SDA_R) return 0;
  SDA_L;
  delay();
  return 1;
}
void IIC_Stop(void) {
  SCL_L;
  delay();
  SDA_L;
  delay();
  SCL_H;
  delay();
  SDA_H;
  delay();
}

void IIC_ACK(void) {
  //SDA_OUT();
  SCL_L;
  delay();
  SDA_L;
  delay();
  SCL_H;
  delay();
  SCL_L;
  delay();

}

void IIC_NACK(void) {
  SCL_L;
  delay();
  SDA_H;
  delay();
  SCL_H;
  delay();
  SCL_L;
  delay();
}

u8 IIC_Wait_ACK(void) {
  //SDA_IN();
  SCL_L;
  delay();
  SDA_H;
  delay();
  SCL_H;
  delay();
  if(SDA_R) {
    SCL_L;
    delay();
    return 0;
  }
  SCL_L;
  delay();
  return 1;

}

void IIC_SendData(u8 data) {
    u8 t;
  //SDA_OUT();
  for(t=0;t<8;t++) {
    SCL_L;
    delay();
    if(data&0x80) {SDA_H;}
    else SDA_L;
    data = (data << 1);
    delay();
    SCL_H;
    delay();
  }
  SCL_L;

}

u8 IIC_ReadData(void) {

  u8 i,r = 0;
  for(i=0;i<8;i++) {
    r <<= 1;
    SCL_L;
    delay();
    SCL_H;
    delay();
    if(SDA_R) r|=0x01;
  }
  SCL_L;
  return r;
}

u8 IIC_Read(u8 address, u8 reg_address) {
  u8 r;
  if(!I2C_Start()) return 0;
  IIC_SendData(address);
  if(!IIC_Wait_ACK()){
    IIC_Stop();
    return 0;
  }
  IIC_SendData(reg_address);
  if(!IIC_Wait_ACK()){
    IIC_Stop();
    return 0;
  }
  I2C_Start();
  IIC_SendData(address+1);
  IIC_Wait_ACK();
  r = IIC_ReadData();
  IIC_NACK();
  IIC_Stop();
  return r;

}


u8 IIC_Write(u8 address, u8 reg_address, u8 data) {
  if(!I2C_Start()) return 0;
  IIC_SendData(address);
  if(!IIC_Wait_ACK()){
    IIC_Stop();
    return 0;
  }
  IIC_SendData(reg_address);
  if(!IIC_Wait_ACK()){
    IIC_Stop();
    return 0;
  }
  IIC_SendData(data);
  if(!IIC_Wait_ACK()){
    IIC_Stop();
    return 0;
  }
  IIC_Stop();
  delay_nms(1);
  return 1;
}

void TIM4_IRQHandler(void)    //    //TIM3�����������ж���Ӧ����
{
      TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  //   //  ����TIM3�����ж���Ӧ������־λ
 //��ʼ����ģʽ����ת��

    flag = (u16)IIC_Read(MPU6050_Addr,0x3D);
    flag <<= 8;
    flag += (u16)IIC_Read(MPU6050_Addr,0x3E);
    if(flag > 0x8000) flag = flag - 0xFFFF;

}
int main(void) {

  RCC_Configuration();
  GPIO_Configuration();
  nvic();
  tim3();
  tim4();
  while(!IIC_Write(MPU6050_Addr,PWR_MGMT_1,0x00));
    delay_nms(500);
    while(!IIC_Write(MPU6050_Addr,CONFIG,0x06));
    while(!IIC_Write(MPU6050_Addr,GYRO_CONFIG,0x18));
    while(!IIC_Write(MPU6050_Addr,ACCEL_CONFIG,0x01));
  while(1) {
    if(flag>16384) {
        flag=16384;

    }
    if(flag<-16384) {
       flag=-16384;
    }
    change(asin(flag/16384.0)/(PI)*1800);
       if(flag>0) {display(r,1);}
       else display(r,0);

  }
}


void tim3()               //����TIM3Ϊ������ʱ��ģʽ ��Լ100Hz
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //������ʽΪTIM_TimeBaseInitTypeDef�Ľṹ��������ΪTIM_TimeBaseStructure

    TIM_TimeBaseStructure. TIM_Period =99;      //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷ�Ƶ����(���������˲�����ʱ����)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //  ��ʼ��tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //����TIM3�����жϱ�־
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  ʹ��TIM3�����������ж�
    TIM_Cmd(TIM3,ENABLE);           //       ʹ��TIM3
}
void tim4()               //����TIM3Ϊ������ʱ��ģʽ ��Լ100Hz
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //������ʽΪTIM_TimeBaseInitTypeDef�Ľṹ��������ΪTIM_TimeBaseStructure

    TIM_TimeBaseStructure. TIM_Period =9999;      //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler =710;     //  ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷ�Ƶ����(���������˲�����ʱ����)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //  ��ʼ��tim3
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //����TIM3�����жϱ�־
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //  ʹ��TIM3�����������ж�
    TIM_Cmd(TIM4,ENABLE);           //       ʹ��TIM3
}
void nvic()                 //�����ж����ȼ�
{
     NVIC_InitTypeDef NVIC_InitStructure;  //  //   ����һ���ȼ�����


     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //Ҫ��ͬһ��Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 ���������ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//  �������ȼ�Ϊ1������һ����ͬ����ϣ���ж��໥���϶Է�
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;   //  ��Ӧ���ȼ�1��������һ�����������ж�ͬʱ��ʱ����һ����ִ��
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //Ҫ��ͬһ��Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //TIM3 ���������ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//  �������ȼ�Ϊ1������һ����ͬ����ϣ���ж��໥���϶Է�
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;   //  ��Ӧ���ȼ�1��������һ�����������ж�ͬʱ��ʱ����һ����ִ��
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
}
void RCC_Configuration(void)				 //ʹ���κ�һ������ʱ�����ؿ�������Ӧ��ʱ��
{

   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOC, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE);
}



void GPIO_Configuration(void)			 //ʹ��ĳio����������ʱ�������ض�����ʼ������
{
    GPIO_InitTypeDef GPIO_InitStructure;   //������ʽΪGPIO_InitTypeDef�Ľṹ��������ΪGPIO_InitStructure

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);   //������ʽΪGPIO_InitTypeDef�Ľṹ��������ΪGPIO_InitStructure

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//ʧ��STM32 JTAG��д���ܣ�ֻ����SWDģʽ��д�����ų�PA15��PB�в���IO��
}
