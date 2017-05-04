#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)

u16 AD_Value;

int counter,flag,counter2;

void RCC_Configuration(void);	 //ʱ�ӳ�ʼ������������ʱ��
void GPIO_Configuration(void);	 //IO�ڳ�ʼ���������书��
void tim3(void);			//��ʱ��tim4��ʼ������
void tim4(void);
void nvic(void);				 //�ж����ȼ�������
void ADC(void);
void dma(void);
void exti(void);

void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

void EXTI2_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET) {
            EXTI_ClearITPendingBit(EXTI_Line2);
            if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)) {
                flag = 1; 
                TIM_Cmd(TIM3,DISABLE); 
            }
            else {
                flag = 0;
								TIM_Cmd(TIM3,ENABLE);
            }
    }     
        /*}
        if(f==1) {
            f = 0;
            //TIM_Cmd(TIM3,DISABLE);
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);
        }
    }*/

    //}
}    

void TIM3_IRQHandler(void)		//	  //TIM3����������ж���Ӧ����
{
	    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	//	 //	 ���TIM3����ж���Ӧ������־λ
	
      GPIO_ResetBits(GPIOC,GPIO_Pin_14);
   
}			

void TIM4_IRQHandler(void) {
	  TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    if(flag) {
        if(AD_Value > 2500) {
          GPIO_SetBits(GPIOC,GPIO_Pin_15);
          counter = 0;
        }
        else {
          counter++;
        }
        if(counter > 2000) {
          GPIO_ResetBits(GPIOC,GPIO_Pin_15);
          counter = 0;
        }
    }
    else {
      GPIO_ResetBits(GPIOC,GPIO_Pin_15);
    }
	}

int main(void)
{//�ó���ΪADC����������

  RCC_Configuration();															   		
  GPIO_Configuration();	
	ADC();
	nvic(); 
	tim3();
  tim4();
	dma();
  exti();
	//GPIO_Write(GPIOC,0x0000);
	
						  
	while(1)
	{
  }		
}	


void RCC_Configuration(void)				 //ʹ���κ�һ������ʱ����ؿ�������Ӧ��ʱ��
{
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO, ENABLE);	  //ʹ��APB2���������ʱ�ӣ�����GPIOA,ADC1, ���ܸ���ʱ��AFIO�ȣ�
                                                                              //�������������裬����̼����ֲ������
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //ʹ��APB1���������ʱ�ӣ���ʱ��tim3��������������ֲ�
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

                
}


void GPIO_Configuration(void)			 //ʹ��ĳio���������ʱ������ض����ʼ������
{
    GPIO_InitTypeDef GPIO_InitStructure;   //�����ʽΪGPIO_InitTypeDef�Ľṹ�������ΪGPIO_InitStructure  
                                       	  //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef;
		GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//ʧ��STM32 JTAG��д���ܣ�ֻ����SWDģʽ��д����ų�PA15��PB�в���IO��
}

void tim4() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //�����ʽΪTIM_TimeBaseInitTypeDef�Ľṹ�������ΪTIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period =999;      //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
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

  	TIM_TimeBaseStructure. TIM_Period =9999;		  //���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
		TIM_TimeBaseStructure.TIM_Prescaler =7100;	   //	 ʱ��Ԥ��Ƶֵ����ʱ���ļ�����Ƶ�ʵ�����ʱ��Ƶ�ʳ��Ը�Ԥ��Ƶֵ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // ʱ�ӷ�Ƶ����(���������˲�����ʱ����)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);		 //	 ��ʼ��tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //���TIM3����жϱ�־
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  ʹ��TIM3����������ж�				  //		   ʹ��TIM3
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

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
     NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
	
 
}
void ADC()
{										 
	ADC_InitTypeDef ADC_InitStructure;
	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   //����ADC1��ADC2�����ڶ�������ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;				// ����ADC������ɨ��ģʽ����ͨ���������ֻ��Ҫһ��ͨ��������Ϊ��ͨ��ģʽ DISABLE
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //����ADC�����ڵ���ģʽ��ʹ��һ�Σ�ת��һ�Σ�Ҳ������������ģʽ���Զ������Բ���
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //����ת������������������ⲿ����
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //ADC�����Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = 1;                          //���ý���ת����ͨ����ĿΪ5
  ADC_Init(ADC1, &ADC_InitStructure);                              //��ʼ��ADC1

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1,ADC_SampleTime_239Cycles5);  //����ADC1��ͨ��0��һ������ת��������ʱ������239.5��
  
  ADC_Cmd(ADC1, ENABLE);                           //ʹ��ADC1
  
  ADC_ResetCalibration(ADC1);                        //����ADC1��У׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1));        //�ȴ�ADC1У׼�������

  ADC_StartCalibration(ADC1);                        //��ʼADC1У׼
  while(ADC_GetCalibrationStatus(ADC1));             //�ȴ�ADC1У׼���
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);   
          //ʹ��ADC1�����ʼת�
}

void dma(void) {
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1,&DMA_InitStructure);
  ADC_DMACmd(ADC1,ENABLE);
  DMA_Cmd(DMA1_Channel1,ENABLE);
}

void exti() {
    EXTI_InitTypeDef EXTI_InitStructure;
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
  
}
