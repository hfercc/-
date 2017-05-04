#include "stm32f10x.h"
#include "stdio.h"
#include "delay.h"
#include "config.h"


void assert_failed(uint8_t* file, uint32_t line)
{
 printf("Wrong parameters value: file %s on line %d\r\n", file, line);
 while(1);
}

   
int main(void)
{
    RCC_Configuration();															   		
    GPIO_Configuration();	
    spi();						
	while(1)
	{ 
	}	
}	
 



