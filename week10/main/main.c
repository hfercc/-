#include "stm32f10x.h"
#include "stdio.h"
#include "stm32_dsp.h"
#include "table_fft.h"
#include <math.h>

#define NPT 1024

int trick = 0;
long lBUFIN[NPT];
long lBUFOUT[NPT];
u16 lBUFMAG[NPT/2];
long lTEMP[NPT];
int max[3];
char test[3] = "123";
int f = 0;
long *base = &lBUFIN[0] - 2;
uint32_t fontaddr = 0;
u8 RR;

#define LCD_DC(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_2);else GPIO_ResetBits(GPIOA,GPIO_Pin_2);}
#define LCD_CS1(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_3);else GPIO_ResetBits(GPIOA,GPIO_Pin_3);}
#define ROM_CS(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_4);else GPIO_ResetBits(GPIOA,GPIO_Pin_4);}
#define SCLK(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_5);else GPIO_ResetBits(GPIOA,GPIO_Pin_5);}
#define SID(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_7);else GPIO_ResetBits(GPIOA,GPIO_Pin_7);}
void delay_nus(int n);
void delay_nms(int n);
void LCD_Init(void);
void TCL(int d);
void TDL(int d);

void D74() {
  int i,j,d;
  for(j = 0;j < 16;j++) {
    d = 1 << (lBUFMAG[i]/500-1);
    if(j<8) {
      for(i=0;i<8;i++) {
        GPIO_ResetBits(GPIOB,GPIO_Pin_0);
        if(d&0x80) {GPIO_SetBits(GPIOB,GPIO_Pin_1);}
        else GPIO_ResetBits(GPIOB,GPIO_Pin_1);;
        GPIO_SetBits(GPIOB,GPIO_Pin_0);
        delay_nus(10);
        d <<= 1;
      }
    }
    else{
      for(i=0;i<8;i++) {
        GPIO_ResetBits(GPIOB,GPIO_Pin_2);
        if(d&0x80) {GPIO_SetBits(GPIOB,GPIO_Pin_3);}
        else GPIO_ResetBits(GPIOB,GPIO_Pin_3);;
        GPIO_SetBits(GPIOB,GPIO_Pin_2);
        delay_nus(10);
        d <<= 1;
      }
    }
  }
}

void RCC_Configuration(void);	 //Ê±ÖÓ³õÊ¼»¯£¬¿ªÆôÍâÉèÊ±ÖÓ
void GPIO_Configuration(void);	 //IO¿Ú³õÊ¼»¯£¬ÅäÖÃÆä¹¦ÄÜ
            //72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±ms
void magdisplay() {
  int i,j,d;
  GPIO_ResetBits(GPIOA,GPIO_Pin_1);
  for(j=0;j<16;j++) {
    GPIO_SetBits(GPIOA,GPIO_Pin_8);
    d = lBUFMAG[j*64];
    GPIO_ResetBits(GPIOB,0x01<<j);
    for(i=0;i<8;i++) {
      GPIO_ResetBits(GPIOA,GPIO_Pin_8);
      if(lBUFMAG[i*128]&0x80) {GPIO_SetBits(GPIOA,GPIO_Pin_9);}
      else GPIO_ResetBits(GPIOA,GPIO_Pin_9);
      GPIO_SetBits(GPIOA,GPIO_Pin_8);
      delay_nus(1);
      d <<= 1;
    }
    GPIO_SetBits(GPIOA,GPIO_Pin_1);
  }
}
void adc(void);
void dma(void);
void nvic(void);
void tim2(void);
void tim3(void);
void dsp_asm_powerMag(void);
void findMax(void);
void Clean(void) {
  int i,j;
  LCD_CS1(0);
  ROM_CS(1);
  for(i=0;i<8;i++) {
    TCL(0xb0+i);
    TCL(0x00);
    TCL(0x10);
    for(j=0;j<128;j++) {
      TDL(0xFF);
    }
  }
  LCD_CS1(1);
}
void SCR(int d) {
  int i;
  for(i=0;i<8;i++) {
    SCLK(0);
    if(d&0x80) {SID(1);}
    else SID(0);
    SCLK(1);
    delay_nus(10);
    d <<= 1;
  }
}
char RCR() {
  int i;
  u8 t=0;
  SCLK(1);;;__nop();
  for(i=0;i<8;i++) {
		;;;__nop();
    SCLK(0);;;__nop();
    t <<= 1;
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)) {
      t+=1;;;__nop();
    }
    SCLK(1);
  }
  return t;
}
void GBR(u8 AH,u8 AM,u8 AL,char *pBuff,u8 L) {
  int i;
  ROM_CS(0);
  LCD_CS1(1);
  SCR(0x03);
  SCR(AH);
  SCR(AM);
  SCR(AL);
  for(i=0;i<L;i++) {
    *(pBuff+i) = RCR();
  }
  ROM_CS(1);
}
void LA(u8 p, u8 c) {
  TCL(0xb0+p);
  TCL((c&0xf0)>>4|0x10);
  TCL((c&0x0f)|0x00);
}

void display_16x16(u8 x,u8 y, char* buf) {
  u32 i,j;
  LCD_CS1(0);
  for(j=2;j>0;j--) {
    LA(x,y);
    for(i=0;i<16;i++) {
      TDL(*buf);
      buf++;
    }
    x++;
  }
  LCD_CS1(1);
}
void display_8x16(u8 x, u8 y, char* buf) {
  u32 i,j;
  LCD_CS1(0);
  for(j=2;j>0;j--) {
    LA(x,y);
    for(i=0;i<8;i++) {
      TDL(*buf);
      buf++;
    }
    x++;
  }
  LCD_CS1(1);
}


void display_string(u8 x, u8 y, char* text) {
  u8 i= 0;
  u8 addrHigh,addrMid,addrLow ;
  char fontbuf[32];
  while((text[i]>0x00))
  {
    if(((text[i]>=0xb0) &&(text[i]<=0xf7))&&(text[i+1]>=0xa1))
    {
      /*¹ú±ê¼òÌå£¨GB2312£©ºº×ÖÔÚ¾§ÁªÑ¶×Ö¿âICÖÐµÄµØÖ·ÓÉÒÔÏÂ¹«Ê½À´¼ÆËã£º*/
      /*Address = ((MSB - 0xB0) * 94 + (LSB - 0xA1)+ 846)*32+ BaseAdd;BaseAdd=0*/
      /*ÓÉÓÚµ£ÐÄ8Î»µ¥Æ¬»úÓÐ³Ë·¨Òç³öÎÊÌâ£¬ËùÒÔ·ÖÈý²¿È¡µØÖ·*/
      fontaddr = ((text[i]- 0xb0)*94 + (text[i+1]-0xA1)+846 )*32;

      addrHigh = (fontaddr&0xff0000)>>16;  /*µØÖ·µÄ¸ß8Î»,¹²24Î»*/
      addrMid = (fontaddr&0xff00)>>8;      /*µØÖ·µÄÖÐ8Î»,¹²24Î»*/
      addrLow = fontaddr&0xff;       /*µØÖ·µÄµÍ8Î»,¹²24Î»*/
      GBR(addrHigh,addrMid,addrLow,fontbuf,32 );/*È¡32¸ö×Ö½ÚµÄÊý¾Ý£¬´æµ½"fontbuf[32]"*/
      display_16x16(x,y,fontbuf);/*ÏÔÊ¾ºº×Öµ½LCDÉÏ£¬yÎªÒ³µØÖ·£¬xÎªÁÐµØÖ·£¬fontbuf[]ÎªÊý¾Ý*/
      i+=2;
      y+=16;
    }
    else if(((text[i]>=0xa1) &&(text[i]<=0xa3))&&(text[i+1]>=0xa1))
    {
      /*¹ú±ê¼òÌå£¨GB2312£©15x16µãµÄ×Ö·ûÔÚ¾§ÁªÑ¶×Ö¿âICÖÐµÄµØÖ·ÓÉÒÔÏÂ¹«Ê½À´¼ÆËã£º*/
      /*Address = ((MSB - 0xa1) * 94 + (LSB - 0xA1))*32+ BaseAdd;BaseAdd=0*/
      /*ÓÉÓÚµ£ÐÄ8Î»µ¥Æ¬»úÓÐ³Ë·¨Òç³öÎÊÌâ£¬ËùÒÔ·ÖÈý²¿È¡µØÖ·*/
      fontaddr = (text[i]- 0xa1)*94;
      fontaddr += (text[i+1]-0xa1);
      fontaddr = (u32)(fontaddr*32);

      addrHigh = (fontaddr&0xff0000)>>16;  /*µØÖ·µÄ¸ß8Î»,¹²24Î»*/
      addrMid = (fontaddr&0xff00)>>8;      /*µØÖ·µÄÖÐ8Î»,¹²24Î»*/
      addrLow = fontaddr&0xff;       /*µØÖ·µÄµÍ8Î»,¹²24Î»*/
      GBR(addrHigh,addrMid,addrLow,fontbuf,32 );/*È¡32¸ö×Ö½ÚµÄÊý¾Ý£¬´æµ½"fontbuf[32]"*/
      display_16x16(y,x,fontbuf);/*ÏÔÊ¾ºº×Öµ½LCDÉÏ£¬yÎªÒ³µØÖ·£¬xÎªÁÐµØÖ·£¬fontbuf[]ÎªÊý¾Ý*/
      i+=2;
      x+=16;
    }
    else if ((text[i]==0xA9)&&(text[i+1]>=0xA1)) {
      fontaddr = (282+(text[i+1]-0xA1)*32);
      addrHigh = (fontaddr&0xff0000)>>16;  /*µØÖ·µÄ¸ß8Î»,¹²24Î»*/
      addrMid = (fontaddr&0xff00)>>8;      /*µØÖ·µÄÖÐ8Î»,¹²24Î»*/
      addrLow = fontaddr&0xff;       /*µØÖ·µÄµÍ8Î»,¹²24Î»*/
      GBR(addrHigh,addrMid,addrLow,fontbuf,32 );/*È¡32¸ö×Ö½ÚµÄÊý¾Ý£¬´æµ½"fontbuf[32]"*/
      display_16x16(y,x,fontbuf);
    }
    else if((text[i]>=0x20) &&(text[i]<=0x7e))
    {
      char fontbuf1[16];
      fontaddr = (text[i]- 0x20);
      fontaddr = (unsigned long)(fontaddr*16);
      fontaddr = (unsigned long)(fontaddr+0x3cf80);
      addrHigh = (fontaddr&0xff0000)>>16;
      addrMid = (fontaddr&0xff00)>>8;
      addrLow = fontaddr&0xff;

      GBR(addrHigh,addrMid,addrLow,fontbuf1,16 );/*È¡16¸ö×Ö½ÚµÄÊý¾Ý£¬´æµ½"fontbuf[32]"*/

      display_8x16(y,x,fontbuf1);/*ÏÔÊ¾8x16µÄASCII×Öµ½LCDÉÏ£¬yÎªÒ³µØÖ·£¬xÎªÁÐµØÖ·£¬fontbuf[]ÎªÊý¾Ý*/
      i+=1;
      x+=8;
    }
    else
      i++;
  }

}
void DMA1_Channel1_IRQHandler(void) {
	DMA_ClearITPendingBit(DMA1_IT_TC1);
	cr4_fft_1024_stm32(lBUFOUT,base,NPT);
	dsp_asm_powerMag();
	findMax();
}
void findMax() {
	int i;
	float a=0;
	float mean[NPT];
	mean[1] = (lBUFMAG[1]);
	mean[2] = (lBUFMAG[1]+lBUFMAG[2]+lBUFMAG[3])/3;
	mean[NPT/2-2] = (lBUFMAG[NPT/2-3]+lBUFMAG[NPT/2-2]+lBUFMAG[NPT/2-1])/3;
	mean[NPT/2-1] = lBUFMAG[NPT/2-1];
	for(i=3;i<NPT/2 - 3;i++) {
		mean[i] = (lBUFMAG[i-2]+lBUFMAG[i-1]+lBUFMAG[i]+lBUFMAG[i+1]+lBUFMAG[i+2])/5;
	}
	a=0;
	for(i=1;i<NPT/2;i++) {
		if(mean[i]>a) {
			max[0] = i;
			a = mean[i];
		}
	}
	a = 0;
	for(i=1;i<NPT/2;i++) {
		if((mean[i]>a)&&(mean[i]<mean[max[0]])) {
			max[1] = i;
			a = mean[i];
		}
	}
	a = 0;
	for(i=1;i<NPT/2;i++) {
		if((mean[i]>a)&&(mean[i]<mean[max[1]])) {
			max[2] = i;
			a = mean[i];
		}
	}

}

void dsp_asm_powerMag(void){
	u32 i;
	s32 lX,lY;
	float X,Y,Mag;
	for(i=0;i<NPT/2;i++)  {
		lX  = (lBUFOUT[i]<<16)>>16;
		lY  = (lBUFOUT[i] >> 16);
		X = (float)lX/512;
		Y = (float)lY/512;
		Mag = sqrt(X*X + Y*Y)/64;
		lBUFMAG[i] = (u16)(Mag*32768);
	}
}

void TIM2_IRQHandler(void) {
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
}

void TIM3_IRQHandler(void) {
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}


int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
    tim2();
    tim3();
    nvic();
	  adc();
	  dma();
    LCD_Init();
    Clean();
	 __nop();;
		display_string(0,0,"1200");
	 while(1){
	};

}



void RCC_Configuration(void)				 //Ê¹ÓÃÈÎºÎÒ»¸öÍâÉèÊ±£¬Îñ±Ø¿ªÆôÆäÏàÓ¦µÄÊ±ÖÓ
{
	RCC_DeInit();
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
    RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3,ENABLE);
}


void GPIO_Configuration(void)			 //Ê¹ÓÃÄ³io¿ÚÊäÈëÊä³öÊ±£¬ÇëÎñ±Ø¶ÔÆä³õÊ¼»¯ÅäÖÃ
{
  GPIO_InitTypeDef GPIO_InitStructure;   //¶¨Òå¸ñÊ½ÎªGPIO_InitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªGPIO_InitStructure
                                       	  //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	   //ÅäÖÃIO¿Ú¹¤×÷Ä£Ê½Îª	ÍÆÍìÊä³ö£¨ÓÐ½ÏÇ¿µÄÊä³öÄÜÁ¦£©
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //ÅäÖÃIO¿Ú×î¸ßµÄÊä³öËÙÂÊÎª50M
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
    		  //³õÊ¼»¯GPIOCµÄÏàÓ¦IO¿ÚÎªÉÏÊöÅäÖÃ£¬ÓÃÓÚledÊä³ö
	GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void tim2(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure

  	TIM_TimeBaseStructure. TIM_Period = 9999;		  //ÅäÖÃ¼ÆÊýãÐÖµÎª9999£¬³¬¹ýÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖÐ¶Ï
	TIM_TimeBaseStructure.TIM_Prescaler =71;	   //	 Ê±ÖÓÔ¤·ÖÆµÖµ£¬¶¨Ê±Æ÷µÄ¼ÆÊýµÄÆµÂÊµÈÓÚÖ÷Ê±ÖÓÆµÂÊ³ýÒÔ¸ÃÔ¤·ÖÆµÖµ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // Ê±ÖÓ·ÖÆµ±¶Êý(ÓÃÓÚÊý×ÖÂË²¨£¬ÔÝÊ±ÎÞÓÃ)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // ¼ÆÊý·½Ê½ÎªÏòÉÏ¼ÆÊý

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);		 //	 ³õÊ¼»¯tim3
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update); //Çå³ýTIM3Òç³öÖÐ¶Ï±êÖ¾
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //  Ê¹ÄÜTIM3µÄÒç³ö¸üÐÂÖÐ¶Ï
    TIM_Cmd(TIM2,ENABLE);
}

void tim3(void) {
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	  //¶¨Òå¸ñÊ½ÎªTIM_TimeBaseInitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªTIM_TimeBaseStructure

  	TIM_TimeBaseStructure. TIM_Period = 195;		  //ÅäÖÃ¼ÆÊýãÐÖµÎª9999£¬³¬¹ýÊ±£¬×Ô¶¯ÇåÁã£¬²¢´¥·¢ÖÐ¶Ï
	 TIM_TimeBaseStructure.TIM_Prescaler =71;	   //	 Ê±ÖÓÔ¤·ÖÆµÖµ£¬¶¨Ê±Æ÷µÄ¼ÆÊýµÄÆµÂÊµÈÓÚÖ÷Ê±ÖÓÆµÂÊ³ýÒÔ¸ÃÔ¤·ÖÆµÖµ
  	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 // Ê±ÖÓ·ÖÆµ±¶Êý(ÓÃÓÚÊý×ÖÂË²¨£¬ÔÝÊ±ÎÞÓÃ)
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	 // ¼ÆÊý·½Ê½ÎªÏòÉÏ¼ÆÊý

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);		 //	 ³õÊ¼»¯tim3
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update); //Çå³ýTIM3Òç³öÖÐ¶Ï±êÖ¾
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //  Ê¹ÄÜTIM3µÄÒç³ö¸üÐÂÖÐ¶Ï
    TIM_Cmd(TIM3,ENABLE);
}

void nvic(void) {
	NVIC_InitTypeDef NVIC_InitStructure;  //	 //	  ÃüÃûÒ»ÓÅÏÈ¼¶±äÁ¿


	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //ÒªÓÃÍ¬Ò»¸öGroup
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM3	Òç³ö¸üÐÂÖÐ¶Ï
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÓëÉÏÒ»¸öÏàÍ¬£¬²»Ï£ÍûÖÐ¶ÏÏà»¥´ò¶Ï¶Ô·½
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	ÏìÓ¦ÓÅÏÈ¼¶1£¬µÍÓÚÉÏÒ»¸ö£¬µ±Á½¸öÖÐ¶ÏÍ¬Ê±À´Ê±£¬ÉÏÒ»¸öÏÈÖ´ÐÐ
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //ÒªÓÃÍ¬Ò»¸öGroup
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3	Òç³ö¸üÐÂÖÐ¶Ï
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//	´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÓëÉÏÒ»¸öÏàÍ¬£¬²»Ï£ÍûÖÐ¶ÏÏà»¥´ò¶Ï¶Ô·½
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 	//	ÏìÓ¦ÓÅÏÈ¼¶1£¬µÍÓÚÉÏÒ»¸ö£¬µ±Á½¸öÖÐ¶ÏÍ¬Ê±À´Ê±£¬ÉÏÒ»¸öÏÈÖ´ÐÐ
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; //TIM3	Òç³ö¸üÐÂÖÐ¶Ï
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//	´ò¶ÏÓÅÏÈ¼¶Îª1£¬ÓëÉÏÒ»¸öÏàÍ¬£¬²»Ï£ÍûÖÐ¶ÏÏà»¥´ò¶Ï¶Ô·½
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 	//	ÏìÓ¦ÓÅÏÈ¼¶1£¬µÍÓÚÉÏÒ»¸ö£¬µ±Á½¸öÖÐ¶ÏÍ¬Ê±À´Ê±£¬ÉÏÒ»¸öÏÈÖ´ÐÐ
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void adc() {
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
}

void dma(void) {
  DMA_InitTypeDef DMA_InitStructure;
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr=(u32)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)lBUFIN;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = NPT;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1,&DMA_InitStructure);
  ADC_DMACmd(ADC1,ENABLE);
  DMA_ITConfig(DMA1_Channel1,DMA_IT_TC,ENABLE);
  DMA_Cmd(DMA1_Channel1,ENABLE);
}

void TCL(int d) {
	int i;
	LCD_DC(0);
	LCD_CS1(0);
	for(i=0;i<8;i++) {
		SCLK(0);
		if(d&0x80) {SID(1);}
		else SID(0);
		SCLK(1);
		delay_nus(10);
		d <<= 1;
	}
	LCD_CS1(1);
}
void TDL(int d) {
  int i;
  LCD_DC(1);
  LCD_CS1(0);
  for(i=0;i<8;i++) {
    SCLK(0);
    if(d&0x80) {SID(1);}
    else SID(0);
    SCLK(1);
    delay_nus(10);
    d <<= 1;
  }
  LCD_CS1(1);
}


void LCD_Init(void) {
	delay_nus(50);
	LCD_CS1(0);
	ROM_CS(1);
	TCL(0xAE);   //display off
	TCL(0x20);	//Set Memory Addressing Mode
	TCL(0x10);	//00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	TCL(0xb0);	//Set Page Start Address for Page Addressing Mode,0-7
	TCL(0xc8);	//Set COM Output Scan Direction
	TCL(0x00);//---set low column address
	TCL(0x10);//---set high column address
	TCL(0x40);//--set start line address
	TCL(0x81);//--set contrast control register
	TCL(0xFF);
	TCL(0xa1);//--set segment re-map 0 to 127
	TCL(0xa6);//--set normal display
	TCL(0xa8);//--set multiplex ratio(1 to 64)
	TCL(0x3F);//
	TCL(0xa4);//0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	TCL(0xd3);//-set display offset
	TCL(0x00);//-not offset
	TCL(0xd5);//--set display clock divide ratio/oscillator frequency
	TCL(0xf0);//--set divide ratio
	TCL(0xd9);//--set pre-charge period
	TCL(0x22); //
	TCL(0xda);//--set com pins hardware configuration
	TCL(0x12);
	TCL(0xdb);//--set vcomh
	TCL(0x20);//0x20,0.77xVcc
	TCL(0x8d);//--set DC-DC enable
	TCL(0x14);//
	TCL(0xaf);//--turn on oled panel
	LCD_CS1(1);
}


void delay_nus(int n)		//72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±us
{
  u8 i;
  while(n--)
  {
    i=7;
    while(i--);
  }
}

void delay_nms(int n)	  //72MÊ±ÖÓÏÂ£¬Ô¼ÑÓÊ±ms
{
    while(n--)
      delay_nus(1000);
}
