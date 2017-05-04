
#include "stm32f10x.h"
#include "math.h"
#include "stdio.h"
u16 counter = 0;
u8 f = 0;
u8 b = 0;
int m=0,nn=0;
int r[4] = {0,0,0,0};
void RCC_Configuration(void);	 //Ê±ÖÓ³õÊ¼»¯£¬¿ªÆôÍâÉèÊ±ÖÓ
void GPIO_Configuration(void);	 //IO¿Ú³õÊ¼»¯£¬ÅäÖÃÆä¹¦ÄÜ
void tim3(void);				 //¶¨Ê±Æ÷tim4³õÊ¼»¯ÅäÖÃ
void tim4(void);				 //¶¨Ê±Æ÷tim4³õÊ¼»¯ÅäÖÃ
void nvic(void);				 //ÖĞ¶ÏÓÅÏÈ¼¶µÈÅäÖÃ
void exti(void);				 //Íâ²¿ÖĞ¶ÏÅäÖÃ
void delay_nus(u32);           //72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±us
void delay_nms(u32);            //72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±ms

int num[4][10] = {{0x01C0,0x01F9,0x01A4,0x01B0,0x0199,0x0192,0x0182,0x01F8,0x0180,0x0190},
                 {0x0240,0x0279,0x0224,0x0230,0x0219,0x0212,0x0202,0x0278,0x0200,0x0210},
                 {0x04C0,0x04F9,0x04A4,0x04B0,0x0499,0x0492,0x0482,0x04F8,0x0480,0x0490},
                 {0x08C0,0x08F9,0x08A4,0x08B0,0x0899,0x0892,0x0882,0x08F8,0x0880,0x0890}};

void display(int n[4]) {
    int i;
    for(i=0;i<4;i++) {
        GPIO_Write(GPIOC,num[i][n[i]]);
    }
}

void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

void change(int n) {
	  float rr = (n*100.0+TIM3->CNT)/1000000*340*1000/2;
    r[3] = rr/1000;
    r[2] = (rr-r[3]*1000)/100;
    r[1] = (rr-r[2]*100-r[3]*1000)/10;
    r[0] = (rr-r[1]*10-r[2]*100-r[3]*1000);
}


void EXTI2_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line2)!=RESET) {
				//GPIO_ResetBits(GPIOC,GPIO_Pin_15);
        /*if(f==0) {
            f = 1;
            //TIM_Cmd(TIM3,ENABLE);*/
						if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2)) {
                                m = 0;
                                TIM3->CNT = 0;		
								TIM_Cmd(TIM3,ENABLE);
						}
						else {
								TIM_Cmd(TIM3,DISABLE);
								change(m);
						}
		}			
        /*}
        if(f==1) {
            f = 0;
            //TIM_Cmd(TIM3,DISABLE);
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);
        }
    }*/
		EXTI_ClearITPendingBit(EXTI_Line2);
    //}
}   

void TIM3_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
    m = m + 1;
}

void TIM4_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
    if(counter < 10) {
        GPIO_SetBits(GPIOA,GPIO_Pin_1);
    }
    else {
        GPIO_ResetBits(GPIOA,GPIO_Pin_1);
    }
		counter++;
    if(counter > 40000) {
        counter = 0;
    }
}	

	


int main(void)
{
	RCC_Configuration();															   		
  GPIO_Configuration();	
	tim4();
	exti(); 
	tim3();
	nvic();
  
						

	while(1)
	{ 
		  display(r);
  }	

}	


void delay_nus(u32 n)		//72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±us
{
  u8 i;
  while(n--)
  {
    i=7;
    while(i--);
  }
}

void delay_nms(u32 n)	  //72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±ms
{
    while(n--)
      delay_nus(1000);
}


void RCC_Configuration(void)				 //Ê¹ÓÃÈÎºÎÒ»¸öÍâÉèÊ±£¬Îñ±Ø¿ªÆôÆäÏàÓ¦µÄÊ±ÖÓ
{
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	  //Ê¹ÄÜAPB2¿ØÖÆÍâÉèµÄÊ±ÖÓ£¬°üÀ¨GPIOC, ¹¦ÄÜ¸´ÓÃÊ±ÖÓAFIOµÈ£¬
                                                                              //ÆäËû°üÀ¨µÄÍâÉè£¬Ïê¼û¹Ì¼ş¿âÊÖ²áµÈ×ÊÁÏ
  
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //Ê¹ÄÜAPB1¿ØÖÆÍâÉèµÄÊ±ÖÓ£¬¶¨Ê±Æ÷tim3¡¢4£¬ÆäËûÍâÉèÏê¼ûÊÖ²á

                
}


void GPIO_Configuration(void)			 //Ê¹ÓÃÄ³io¿ÚÊäÈëÊä³öÊ±£¬ÇëÎñ±Ø¶ÔÆä³õÊ¼»¯ÅäÖÃ
{
    GPIO_InitTypeDef GPIO_InitStructure;   //¶¨Òå¸ñÊ½ÎªGPIO_InitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªGPIO_InitStructure  
    

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //ÅäÖÃIO¿Ú¹¤×÷Ä£Ê½Îª	ÍÆÍìÊä³ö£¨ÓĞ½ÏÇ¿µÄÊä³öÄÜÁ¦£©
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //ÅäÖÃIO¿Ú×î¸ßµÄÊä³öËÙÂÊÎª50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 //ÅäÖÃ±»Ñ¡ÖĞµÄ¹Ü½Å£¬|±íÊ¾Í¬Ê±±»Ñ¡ÖĞ
    GPIO_Init(GPIOA, &GPIO_InitStructure);		  //³õÊ¼»¯GPIOCµÄÏàÓ¦IO¿ÚÎªÉÏÊöÅäÖÃ£¬ÓÃÓÚledÊä³ö

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//Ê§ÄÜSTM32 JTAGÉÕĞ´¹¦ÄÜ£¬Ö»ÄÜÓÃSWDÄ£Ê½ÉÕĞ´£¬½â·Å³öPA15ºÍPBÖĞ²¿·ÖIO¿Ú
}

void tim4()							  //ÅäÖÃTIM4Îª»ù±¾¶¨Ê±Æ÷Ä£Ê½,Ô¼10ms´¥·¢Ò»´Î£¬´¥·¢ÆµÂÊÔ¼100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period = 9;		  // ÅäÖÃ¼ÆÊıãĞÖµÎª9999£¬³¬¹ıÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖĞ¶Ï
		TIM_TimeBaseStructure.TIM_Prescaler =71;		 //	 Ê±ÖÓÔ¤·ÖÆµÖµ£¬³ıÒÔ¶àÉÙ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// Ê±ÖÓ·ÖÆµ±¶Êı
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// ¼ÆÊı·½Ê½ÎªÏòÉÏ¼ÆÊı

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);		 //	 ³õÊ¼»¯tim4
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //Çå³ıTIM4Òç³öÖĞ¶Ï±êÖ¾
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);   //  Ê¹ÄÜTIM4µÄÒç³ö¸üĞÂÖĞ¶Ï
    TIM_Cmd(TIM4,ENABLE);				 //		   Ê¹ÄÜTIM4
}

void tim3() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period = 99;     // ÅäÖÃ¼ÆÊıãĞÖµÎª9999£¬³¬¹ıÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖĞ¶Ï
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  Ê±ÖÓÔ¤·ÖÆµÖµ£¬³ıÒÔ¶àÉÙ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // Ê±ÖÓ·ÖÆµ±¶Êı
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ¼ÆÊı·½Ê½ÎªÏòÉÏ¼ÆÊı

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //  ³õÊ¼»¯tim4
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //Çå³ıTIM4Òç³öÖĞ¶Ï±êÖ¾
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);   //  Ê¹ÄÜTIM4µÄÒç³ö¸üĞÂÖĞ¶Ï
    TIM_Cmd(TIM3,DISABLE);   
}
void nvic()									//ÅäÖÃÖĞ¶ÏÓÅÏÈ¼¶
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  ÃüÃûÒ»ÓÅÏÈ¼¶±äÁ¿

 	   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    //	   ½«ÓÅÏÈ¼¶·Ö×é·½Ê½ÅäÖÃÎªgroup1,ÓĞ2¸öÇÀÕ¼£¨´ò¶Ï£©ÓÅÏÈ¼¶£¬8¸öÏìÓ¦ÓÅÏÈ¼¶
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //¸ÃÖĞ¶ÏÎªTIM4Òç³ö¸üĞÂÖĞ¶Ï
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÔÚ¸Ã×éÖĞÎª½ÏµÍµÄ£¬0ÓÅÏÈ¼¶×î¸ß
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // ÏìÓ¦ÓÅÏÈ¼¶0£¬´ò¶ÏÓÅÏÈ¼¶Ò»ÑùÊ±£¬0×î¸ß
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 	//	ÉèÖÃÊ¹ÄÜ
     NVIC_Init(&NVIC_InitStructure);					   	//	³õÊ¼»

     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
     NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

     NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
     NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; 
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);

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
