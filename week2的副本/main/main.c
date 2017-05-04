
#include "stm32f10x.h"
#include "math.h"
#include "stdio.h"
u16 counter = 0;
u8 f = 0;
u8 b = 0;
int m=0,nn=0;
int r[4] = {0,0,0,0};
void RCC_Configuration(void);	 //ʱ�ӳ�ʼ������������ʱ��
void GPIO_Configuration(void);	 //IO�ڳ�ʼ���������书��
void tim3(void);				 //��ʱ��tim4��ʼ������
void tim4(void);				 //��ʱ��tim4��ʼ������
void nvic(void);				 //�ж����ȼ�������
void exti(void);				 //�ⲿ�ж�����
void delay_nus(u32);           //72Mʱ���£�Լ��ʱus
void delay_nms(u32);            //72Mʱ���£�Լ��ʱms

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
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	  //ʹ��APB2���������ʱ�ӣ�����GPIOC, ���ܸ���ʱ��AFIO�ȣ�
                                                                              //�������������裬����̼����ֲ������
  
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //ʹ��APB1���������ʱ�ӣ���ʱ��tim3��4��������������ֲ�

                
}


void GPIO_Configuration(void)			 //ʹ��ĳio���������ʱ������ض����ʼ������
{
    GPIO_InitTypeDef GPIO_InitStructure;   //�����ʽΪGPIO_InitTypeDef�Ľṹ�������ΪGPIO_InitStructure  
    

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //����IO�ڹ���ģʽΪ	����������н�ǿ�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //����IO����ߵ��������Ϊ50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 //���ñ�ѡ�еĹܽţ�|��ʾͬʱ��ѡ��
    GPIO_Init(GPIOA, &GPIO_InitStructure);		  //��ʼ��GPIOC����ӦIO��Ϊ�������ã�����led���

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//ʧ��STM32 JTAG��д���ܣ�ֻ����SWDģʽ��д����ų�PA15��PB�в���IO��
}

void tim4()							  //����TIM4Ϊ������ʱ��ģʽ,Լ10ms����һ�Σ�����Ƶ��Լ100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //�����ʽΪTIM_TimeBaseInitTypeDef�Ľṹ�������ΪTIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period = 9;		  // ���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
		TIM_TimeBaseStructure.TIM_Prescaler =71;		 //	 ʱ��Ԥ��Ƶֵ�����Զ���
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// ʱ�ӷ�Ƶ����
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);		 //	 ��ʼ��tim4
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //���TIM4����жϱ�־
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);   //  ʹ��TIM4����������ж�
    TIM_Cmd(TIM4,ENABLE);				 //		   ʹ��TIM4
}

void tim3() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //�����ʽΪTIM_TimeBaseInitTypeDef�Ľṹ�������ΪTIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period = 99;     // ���ü�����ֵΪ9999������ʱ���Զ����㣬�������ж�
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  ʱ��Ԥ��Ƶֵ�����Զ���
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ʱ�ӷ�Ƶ����
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ������ʽΪ���ϼ���

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //  ��ʼ��tim4
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //���TIM4����жϱ�־
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);   //  ʹ��TIM4����������ж�
    TIM_Cmd(TIM3,DISABLE);   
}
void nvic()									//�����ж����ȼ�
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  ����һ���ȼ�����

 	   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    //	   �����ȼ����鷽ʽ����Ϊgroup1,��2����ռ����ϣ����ȼ���8����Ӧ���ȼ�
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //���ж�ΪTIM4��������ж�
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//������ȼ�Ϊ1���ڸ�����Ϊ�ϵ͵ģ�0���ȼ����
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // ��Ӧ���ȼ�0��������ȼ�һ��ʱ��0���
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 	//	����ʹ��
     NVIC_Init(&NVIC_InitStructure);					   	//	��ʼ�

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
