
#include "stm32f10x.h"
#include "math.h"
#include "stdio.h"
u16 counter = 0;
u8 f = 0;
u8 b = 0;
int m=0,nn=0;
int r[4] = {0,0,0,0};
void RCC_Configuration(void);	 //时钟初始化，开启外设时钟
void GPIO_Configuration(void);	 //IO口初始化，配置其功能
void tim3(void);				 //定时器tim4初始化配置
void tim4(void);				 //定时器tim4初始化配置
void nvic(void);				 //中断优先级等配置
void exti(void);				 //外部中断配置
void delay_nus(u32);           //72M时钟下，约延时us
void delay_nms(u32);            //72M时钟下，约延时ms

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
  
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);	  //使能APB2控制外设的时钟，包括GPIOC, 功能复用时钟AFIO等，
                                                                              //其他包括的外设，详见固件库手册等资料
  
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM3, ENABLE); //使能APB1控制外设的时钟，定时器tim3、4，其他外设详见手册

                
}


void GPIO_Configuration(void)			 //使用某io口输入输出时，请务必对其初始化配置
{
    GPIO_InitTypeDef GPIO_InitStructure;   //定义格式为GPIO_InitTypeDef的结构体的名字为GPIO_InitStructure  
    

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC,&GPIO_InitStructure);


	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	   //配置IO口工作模式为	推挽输出（有较强的输出能力）
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //配置IO口最高的输出速率为50M
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;	 //配置被选中的管脚，|表示同时被选中
    GPIO_Init(GPIOA, &GPIO_InitStructure);		  //初始化GPIOC的相应IO口为上述配置，用于led输出

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);	//失能STM32 JTAG烧写功能，只能用SWD模式烧写，解放出PA15和PB中部分IO口
}

void tim4()							  //配置TIM4为基本定时器模式,约10ms触发一次，触发频率约100Hz
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //定义格式为TIM_TimeBaseInitTypeDef的结构体的名字为TIM_TimeBaseStructure  

  	TIM_TimeBaseStructure. TIM_Period = 9;		  // 配置计数阈值为9999，超过时，自动清零，并触发中断
		TIM_TimeBaseStructure.TIM_Prescaler =71;		 //	 时钟预分频值，除以多少
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	// 时钟分频倍数
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	// 计数方式为向上计数

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);		 //	 初始化tim4
    TIM_ClearITPendingBit(TIM4,TIM_IT_Update); //清除TIM4溢出中断标志
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);   //  使能TIM4的溢出更新中断
    TIM_Cmd(TIM4,ENABLE);				 //		   使能TIM4
}

void tim3() {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;   //定义格式为TIM_TimeBaseInitTypeDef的结构体的名字为TIM_TimeBaseStructure  

    TIM_TimeBaseStructure. TIM_Period = 99;     // 配置计数阈值为9999，超过时，自动清零，并触发中断
    TIM_TimeBaseStructure.TIM_Prescaler =71;     //  时钟预分频值，除以多少
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频倍数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 计数方式为向上计数

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);    //  初始化tim4
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //清除TIM4溢出中断标志
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);   //  使能TIM4的溢出更新中断
    TIM_Cmd(TIM3,DISABLE);   
}
void nvic()									//配置中断优先级
{	 
     NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  命名一优先级变量

 	   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    //	   将优先级分组方式配置为group1,有2个抢占（打断）优先级，8个响应优先级
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn; //该中断为TIM4溢出更新中断
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//打断优先级为1，在该组中为较低的，0优先级最高
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 响应优先级0，打断优先级一样时，0最高
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	 	//	设置使能
     NVIC_Init(&NVIC_InitStructure);					   	//	初始�

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
