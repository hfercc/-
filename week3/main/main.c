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
    u16 num[10] = {0xF1C0,0xF1F9,0xF1A4,0xF1B0,0xF199,0xF192,0xF182,0xF1F8,0xF180,0xF190};
		u8 n=0;
    RCC_Configuration();															   		
    GPIO_Configuration();							
		GPIO_Write(GPIOC,num[n]);
	  
	while(1)
	{ 
        GPIO_SetBits(GPIOA,GPIO_Pin_0);
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
    GPIO_Init(GPIOC, &GPIO_InitStructure);		  //��ʼ��GPIOC����ӦIO��Ϊ�������ã�����led���
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //����IO�ڹ���ģʽΪ	����������н�ǿ�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //����IO����ߵ��������Ϊ50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
    

}

