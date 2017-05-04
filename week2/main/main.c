#include "stm32f10x.h"
#include "stdio.h"


void RCC_Configuration(void);	 //ʱ�ӳ�ʼ������������ʱ��
void GPIO_Configuration(void);	 //IO�ڳ�ʼ���������书��
void delay_nus(u32);           //72Mʱ���£�Լ��ʱus
void delay_nms(u32);            //72Mʱ���£�Լ��ʱms

void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

   
int main(void)
{
    //u16 num[10] = {0xF1C0,0xF1F9,0xF1A4,0xF1B0,0xF199,0xF192,0xF182,0xF1F8,0xF180,0xF190};
		u16 num[4][4] = {{0x0001,0x0002,0x0004,0x0008},{0x0005,0x000A,0x0005,0x000A},{0x0001,0x0004,0x0002,0x0008},{0x0008,0x0004,0x0002,0x0001}};
		u8 sp[4] = { 10,30,50,70 };
		u8 m=0,n=0;
		u8 i=0;
    RCC_Configuration();															   		
    GPIO_Configuration();							
		GPIO_Write(GPIOC,0xF000);
	  
	while(1)
	{ 
		 u8 p12,p13;
		 while(i<4) {
			   
		     delay_nms(sp[n]);
		     GPIO_Write(GPIOA,num[m][i]);
		     delay_nms(sp[n]);
		     GPIO_Write(GPIOA,num[m][i]);
			   i = i + 1;
			   if(i==4) i=0;
			   p12 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_12);
		 p13 = GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
		 
		 if (!p13) {
			 m = m + 1;
			 if(m>3) m=0;		 
		 }
		 if (!p12) {
			 
			 n = n + 1;
			 if(n>3) n=0;
		 }
		 }
		 
   }	

}	
 

void delay_nus(u32 n)		//72Mʱ���£�Լ��ʱus
{
  u8 i;
  while(n--)
  {
    i=7;
    while(i--);
  }
}

void delay_nms(u32 n)	  //72Mʱ���£�Լ��ʱms
{
    while(n--)
      delay_nus(1000);
}

void RCC_Configuration(void)				 //ʹ���κ�һ������ʱ����ؿ�������Ӧ��ʱ��
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);	  //ʹ��APB2���������ʱ�ӣ�����GPIOC               
}


void GPIO_Configuration(void)			 //ʹ��ĳio���������ʱ������ض����ʼ������
{
    GPIO_InitTypeDef GPIO_InitStructure;   //�����ʽΪGPIO_InitTypeDef�Ľṹ�������ΪGPIO_InitStructure  
                                       	  //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef;

		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //����IO�ڹ���ģʽΪ	����������н�ǿ�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //����IO����ߵ��������Ϊ50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    		  //��ʼ��GPIOC����ӦIO��Ϊ�������ã�����led���
	  GPIO_Init(GPIOA, &GPIO_InitStructure);	
		GPIO_Init(GPIOC, &GPIO_InitStructure);

}