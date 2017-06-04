#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"
#include "config.h"
#define SCLK(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_5);else GPIO_ResetBits(GPIOA,GPIO_Pin_5);}
#define SID(a) {if(a) GPIO_SetBits(GPIOA,GPIO_Pin_7);else GPIO_ResetBits(GPIOA,GPIO_Pin_7);}


void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
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


int main(void)
{
    RCC_Configuration();
    GPIO_Configuration();
	while(1)
	{
	}
}
