#include "stm32f10x.h"
#include "stdio.h"


void RCC_Configuration(void);	 //时钟初始化，开启外设时钟
void GPIO_Configuration(void);	 //IO口初始化，配置其功能
void delay_nus(u32);           //72M时钟下，约延时us
void delay_nms(u32);            //72M时钟下，约延时ms

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
 

void delay_nus(u32 n)		//72M时钟下，约延时us
{
  u8 i;
  while(n--)
  {
    i=7;
    while(i--);
  }
}

void delay_nms(u32 n)	  //72M时钟下，约延时ms
{
    while(n--)
      delay_nus(1000);
}

void RCC_Configuration(void)				 //使用任何一个外设时，务必开启其相应的时钟
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);	  //使能APB2控制外设的时钟，包括GPIOC               
}


void GPIO_Configuration(void)			 //使用某io口输入输出时，请务必对其初始化配置
{
    GPIO_InitTypeDef GPIO_InitStructure;   //定义格式为GPIO_InitTypeDef的结构体的名字为GPIO_InitStructure  
                                       	  //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef;

		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //配置IO口工作模式为	推挽输出（有较强的输出能力）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //配置IO口最高的输出速率为50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC, &GPIO_InitStructure);		  //初始化GPIOC的相应IO口为上述配置，用于led输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //配置IO口工作模式为	推挽输出（有较强的输出能力）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //配置IO口最高的输出速率为50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOA, &GPIO_InitStructure);	
    

}

