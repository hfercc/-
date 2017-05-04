
#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"


u8 RV[3];
u8 counter = 0;


void RCC_Configuration(void);	 //时钟初始化，开启外设时钟
void GPIO_Configuration(void);	 //IO口初始化，配置其功能
void tim3(void);			//定时器tim4初始化配置
void tim4(void);
void nvic(void);				 //中断优先级等配置
void dma(void);
void usart(void);

void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

/*void USART2_IRQHandler(void) {
    USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    u16 len;
    u16 i;

}*/

 


void TIM3_IRQHandler(void)		//	  //TIM3的溢出更新中断响应函数
{
      int i = 0;
	    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	//	 //	 清空TIM3溢出中断响应函数标志位
 //开始单次模式数据转换
      counter++;
      if(counter>254) counter = 0;
      for(i=0;i<3;i++) {
          if(counter < RV[i]) {
              GPIO_SetBits(GPIOA,1<<(8+i));
          }
          else {
              GPIO_ResetBits(GPIOA,1<<(8+i));
          }
      }
   
}			

void TIM4_IRQHandler(void) {
    int i = 0;
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    for(i = 0;i < 3;i++) {
      USART_SendData(USART2,RV[i]);
      while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET);
    }
}


int main(void)
{//该程序为ADC采样的例程

  RCC_Configuration();															   		
  GPIO_Configuration();	
	//nvic(); 
	//tim3();
  //tim4();
	//usart();
	//dma();
	
						  
	while(1)
	{
        GPIO_SetBits(GPIOA,GPIO_Pin_8);
   }		
}	


void RCC_Configuration(void)				 //使用任何一个外设时，务必开启其相应的时钟
{
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO, ENABLE);	  //使能APB2控制外设的时钟，包括GPIOA,ADC1, 功能复用时钟AFIO等，
                                                                              //其他包括的外设，详见固件库手册等资料
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //使能APB1控制外设的时钟，定时器tim3，其他外设详见手册
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
                
}


void GPIO_Configuration(void)			 //使用某io口输入输出时，请务必对其初始化配置
{
    GPIO_InitTypeDef GPIO_InitStructure;   //定义格式为GPIO_InitTypeDef的结构体的名字为GPIO_InitStructure  
                                       	  //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//失能STM32 JTAG烧写功能，只能用SWD模式烧写，解放出PA15和PB中部分IO口
}

void tim4() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //定义格式为TIM_TimeBaseInitTypeDef的结构体的名字为TIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period =9999;      //配置计数阈值为9999，超过时，自动清零，并触发中断
    TIM_TimeBaseStructure.TIM_Prescaler =7100;     //  时钟预分频值，定时器的计数的频率等于主时钟频率除以该预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频倍数(用于数字滤波，暂时无用)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // 计数方式为向上计数

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //  初始化tim3
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除TIM3溢出中断标志
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //  使能TIM3的溢出更新中断
    TIM_Cmd(TIM4,ENABLE);          
}

void tim3()							  //配置TIM3为基本定时器模式 ，约100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //定义格式为TIM_TimeBaseInitTypeDef的结构体的名字为TIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period =99;		  //配置计数阈值为9999，超过时，自动清零，并触发中断
		TIM_TimeBaseStructure.TIM_Prescaler =71;	   //	 时钟预分频值，定时器的计数的频率等于主时钟频率除以该预分频值
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // 时钟分频倍数(用于数字滤波，暂时无用)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // 计数方式为向上计数

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);		 //	 初始化tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //清除TIM3溢出中断标志
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  使能TIM3的溢出更新中断
    TIM_Cmd(TIM3,ENABLE);					  //		   使能TIM3
}


void nvic()									//配置中断优先级
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  命名一优先级变量

	 
		 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //要用同一个Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3	溢出更新中断
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	打断优先级为1，与上一个相同，不希望中断相互打断对方
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	响应优先级1，低于上一个，当两个中断同时来时，上一个先执行
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //要用同一个Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //TIM3 溢出更新中断
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//  打断优先级为1，与上一个相同，不希望中断相互打断对方
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   //  响应优先级1，低于上一个，当两个中断同时来时，上一个先执行
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
	   
     /*NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);*/


 
}

void dma(void) {
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&USART2->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)RV;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6,&DMA_InitStructure);
  USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
  DMA_Cmd(DMA1_Channel6,ENABLE);
}

void usart(void) {
  
  USART_InitTypeDef USART_InitStructure;
	USART_DeInit(USART2);
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength= USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2,&USART_InitStructure);

  USART_Cmd(USART2,ENABLE);
}

