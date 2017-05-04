
#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"


u8 RV[3];
u8 counter = 0;


void RCC_Configuration(void);	 //ʱ�ӳ�ʼ������������ʱ��
void GPIO_Configuration(void);	 //IO�ڳ�ʼ���������书��
void tim3(void);			//��ʱ��tim4��ʼ������
void tim4(void);
void nvic(void);				 //�ж����ȼ�������
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

 


void TIM3_IRQHandler(void)		//	  //TIM3����������ж���Ӧ����
{
      int i = 0;
	    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	//	 //	 ���TIM3����ж���Ӧ������־λ
 //��ʼ����ģʽ����ת��
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
{//�ó���ΪADC����������

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


void RCC_Configuration(void)				 //ʹ���κ�һ������ʱ����ؿ�������Ӧ��ʱ��
{
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO, ENABLE);	  //ʹ��APB2���������ʱ�ӣ�����GPIOA,ADC1, ���ܸ���ʱ��AFIO�ȣ�
                                                                              //�������������裬����̼����ֲ������
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //ʹ��APB1���������ʱ�ӣ���ʱ��tim3��������������ֲ�
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
                
}


void GPIO_Configuration(void)			 //ʹ��ĳio���������ʱ������ض����ʼ������
{
    GPIO_InitTypeDef GPIO_InitStructure;   //�����ʽΪGPIO_InitTypeDef�Ľṹ�������ΪGPIO_InitStructure  
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


    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//ʧ��STM32 JTAG��д���ܣ�ֻ����SWDģʽ��д����ų�PA15��PB�в���IO��
}

void tim4() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //�����ʽΪTIM_TimeBaseInitTypeDef�Ľṹ�������ΪTIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period =9999;      //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler =7100;     //  ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷ�Ƶ����(���������˲�����ʱ����)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //  ��ʼ��tim3
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //���TIM3����жϱ�־
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //  ʹ��TIM3����������ж�
    TIM_Cmd(TIM4,ENABLE);          
}

void tim3()							  //����TIM3Ϊ������ʱ��ģʽ ��Լ100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //�����ʽΪTIM_TimeBaseInitTypeDef�Ľṹ�������ΪTIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period =99;		  //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
		TIM_TimeBaseStructure.TIM_Prescaler =71;	   //	 ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // ʱ�ӷ�Ƶ����(���������˲�����ʱ����)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);		 //	 ��ʼ��tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //���TIM3����жϱ�־
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  ʹ��TIM3����������ж�
    TIM_Cmd(TIM3,ENABLE);					  //		   ʹ��TIM3
}


void nvic()									//�����ж����ȼ�
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  ����һ���ȼ�����

	 
		 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //Ҫ��ͬһ��Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3	��������ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	������ȼ�Ϊ1������һ����ͬ����ϣ���ж��໥��϶Է�
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	��Ӧ���ȼ�1��������һ�����������ж�ͬʱ��ʱ����һ����ִ��
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //Ҫ��ͬһ��Group
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //TIM3 ��������ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//  ������ȼ�Ϊ1������һ����ͬ����ϣ���ж��໥��϶Է�
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   //  ��Ӧ���ȼ�1��������һ�����������ж�ͬʱ��ʱ����һ����ִ��
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

