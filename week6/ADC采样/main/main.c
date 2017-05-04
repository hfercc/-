#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"

#define ADC1_DR_Address    ((uint32_t)0x4001244C)

u16 AD_Value;

int counter,flag,counter2;

void RCC_Configuration(void);	 //Ê±ÖÓ³õÊ¼»¯£¬¿ªÆôÍâÉèÊ±ÖÓ
void GPIO_Configuration(void);	 //IO¿Ú³õÊ¼»¯£¬ÅäÖÃÆä¹¦ÄÜ
void tim3(void);			//¶¨Ê±Æ÷tim4³õÊ¼»¯ÅäÖÃ
void tim4(void);
void nvic(void);				 //ÖÐ¶ÏÓÅÏÈ¼¶µÈÅäÖÃ
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

void TIM3_IRQHandler(void)		//	  //TIM3µÄÒç³ö¸üÐÂÖÐ¶ÏÏìÓ¦º¯Êý
{
	    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	//	 //	 Çå¿ÕTIM3Òç³öÖÐ¶ÏÏìÓ¦º¯Êý±êÖ¾Î»
	
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
{//¸Ã³ÌÐòÎªADC²ÉÑùµÄÀý³Ì

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


void RCC_Configuration(void)				 //Ê¹ÓÃÈÎºÎÒ»¸öÍâÉèÊ±£¬Îñ±Ø¿ªÆôÆäÏàÓ¦µÄÊ±ÖÓ
{
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_ADC1|RCC_APB2Periph_AFIO, ENABLE);	  //Ê¹ÄÜAPB2¿ØÖÆÍâÉèµÄÊ±ÖÓ£¬°üÀ¨GPIOA,ADC1, ¹¦ÄÜ¸´ÓÃÊ±ÖÓAFIOµÈ£¬
                                                                              //ÆäËû°üÀ¨µÄÍâÉè£¬Ïê¼û¹Ì¼þ¿âÊÖ²áµÈ×ÊÁÏ
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //Ê¹ÄÜAPB1¿ØÖÆÍâÉèµÄÊ±ÖÓ£¬¶¨Ê±Æ÷tim3£¬ÆäËûÍâÉèÏê¼ûÊÖ²á
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

                
}


void GPIO_Configuration(void)			 //Ê¹ÓÃÄ³io¿ÚÊäÈëÊä³öÊ±£¬ÇëÎñ±Ø¶ÔÆä³õÊ¼»¯ÅäÖÃ
{
    GPIO_InitTypeDef GPIO_InitStructure;   //¶¨Òå¸ñÊ½ÎªGPIO_InitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªGPIO_InitStructure  
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

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//Ê§ÄÜSTM32 JTAGÉÕÐ´¹¦ÄÜ£¬Ö»ÄÜÓÃSWDÄ£Ê½ÉÕÐ´£¬½â·Å³öPA15ºÍPBÖÐ²¿·ÖIO¿Ú
}

void tim4() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period =999;      //ÅäÖÃ¼ÆÊýãÐÖµÎª9999£¬³¬¹ýÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖÐ¶Ï
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  Ê±ÖÓÔ¤·ÖÆµÖµ£¬¶¨Ê±Æ÷µÄ¼ÆÊýµÄÆµÂÊµÈÓÚÖ÷Ê±ÖÓÆµÂÊ³ýÒÔ¸ÃÔ¤·ÖÆµÖµ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // Ê±ÖÓ·ÖÆµ±¶Êý(ÓÃÓÚÊý×ÖÂË²¨£¬ÔÝÊ±ÎÞÓÃ)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  // ¼ÆÊý·½Ê½ÎªÏòÉÏ¼ÆÊý

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //  ³õÊ¼»¯tim3
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //Çå³ýTIM3Òç³öÖÐ¶Ï±êÖ¾
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //  Ê¹ÄÜTIM3µÄÒç³ö¸üÐÂÖÐ¶Ï
    TIM_Cmd(TIM4,ENABLE);          
}

void tim3()							  //ÅäÖÃTIM3Îª»ù±¾¶¨Ê±Æ÷Ä£Ê½ £¬Ô¼100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period =9999;		  //ÅäÖÃ¼ÆÊýãÐÖµÎª9999£¬³¬¹ýÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖÐ¶Ï
		TIM_TimeBaseStructure.TIM_Prescaler =7100;	   //	 Ê±ÖÓÔ¤·ÖÆµÖµ£¬¶¨Ê±Æ÷µÄ¼ÆÊýµÄÆµÂÊµÈÓÚÖ÷Ê±ÖÓÆµÂÊ³ýÒÔ¸ÃÔ¤·ÖÆµÖµ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // Ê±ÖÓ·ÖÆµ±¶Êý(ÓÃÓÚÊý×ÖÂË²¨£¬ÔÝÊ±ÎÞÓÃ)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // ¼ÆÊý·½Ê½ÎªÏòÉÏ¼ÆÊý

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);		 //	 ³õÊ¼»¯tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //Çå³ýTIM3Òç³öÖÐ¶Ï±êÖ¾
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  Ê¹ÄÜTIM3µÄÒç³ö¸üÐÂÖÐ¶Ï				  //		   Ê¹ÄÜTIM3
}


void nvic()									//ÅäÖÃÖÐ¶ÏÓÅÏÈ¼¶
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  ÃüÃûÒ»ÓÅÏÈ¼¶±äÁ¿

	 
		 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //ÒªÓÃÍ¬Ò»¸öGroup
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3	Òç³ö¸üÐÂÖÐ¶Ï
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÓëÉÏÒ»¸öÏàÍ¬£¬²»Ï£ÍûÖÐ¶ÏÏà»¥´ò¶Ï¶Ô·½
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	ÏìÓ¦ÓÅÏÈ¼¶1£¬µÍÓÚÉÏÒ»¸ö£¬µ±Á½¸öÖÐ¶ÏÍ¬Ê±À´Ê±£¬ÉÏÒ»¸öÏÈÖ´ÐÐ
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //ÒªÓÃÍ¬Ò»¸öGroup
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //TIM3 Òç³ö¸üÐÂÖÐ¶Ï
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//  ´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÓëÉÏÒ»¸öÏàÍ¬£¬²»Ï£ÍûÖÐ¶ÏÏà»¥´ò¶Ï¶Ô·½
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;   //  ÏìÓ¦ÓÅÏÈ¼¶1£¬µÍÓÚÉÏÒ»¸ö£¬µ±Á½¸öÖÐ¶ÏÍ¬Ê±À´Ê±£¬ÉÏÒ»¸öÏÈÖ´ÐÐ
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
	
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   //ÉèÖÃADC1ºÍADC2¹¤×÷ÔÚ¶ÀÁ¢¹¤×÷Ä£Ê½
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;				// ÉèÖÃADC¹¤×÷ÔÚÉ¨ÃèÄ£Ê½£¨¶àÍ¨µÀ£©£¬Èç¹ûÖ»ÐèÒªÒ»¸öÍ¨µÀ£¬ÅäÖÃÎªµ¥Í¨µÀÄ£Ê½ DISABLE
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;  //ÉèÖÃADC¹¤×÷ÔÚµ¥´ÎÄ£Ê½£¨Ê¹ÄÜÒ»´Î£¬×ª»»Ò»´Î£»Ò²¿ÉÒÔÅäÖÃÁ¬ÐøÄ£Ê½£¬×Ô¶¯ÖÜÆÚÐÔ²ÉÑù
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //ÉèÖÃ×ª»»ÓÉÈí¼þ´¥·¢£¬²»ÓÃÍâ²¿´¥·¢
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              //ADCÊý¾ÝÓÒ¶ÔÆë
  ADC_InitStructure.ADC_NbrOfChannel = 1;                          //ÉèÖÃ½øÐÐ×ª»»µÄÍ¨µÀÊýÄ¿Îª5
  ADC_Init(ADC1, &ADC_InitStructure);                              //³õÊ¼»¯ADC1

  ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1,ADC_SampleTime_239Cycles5);  //ÉèÖÃADC1µÄÍ¨µÀ0µÚÒ»¸ö½øÐÐ×ª»»£¬²ÉÑùÊ±ÖÓÖÜÆÚ239.5¸ö
  
  ADC_Cmd(ADC1, ENABLE);                           //Ê¹ÄÜADC1
  
  ADC_ResetCalibration(ADC1);                        //ÖØÖÃADC1µÄÐ£×¼¼Ä´æÆ÷
  while(ADC_GetResetCalibrationStatus(ADC1));        //µÈ´ýADC1Ð£×¼ÖØÖÃÍê³É

  ADC_StartCalibration(ADC1);                        //¿ªÊ¼ADC1Ð£×¼
  while(ADC_GetCalibrationStatus(ADC1));             //µÈ´ýADC1Ð£×¼Íê³É
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);   
          //Ê¹ÄÜADC1Èí¼þ¿ªÊ¼×ª»
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
