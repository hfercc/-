#include "stm32f10x.h"

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
