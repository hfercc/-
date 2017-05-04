//////////////////////////////////////////////////////////////////////////////////	 
//±¾³ÌÐòÖ»¹©Ñ§Ï°Ê¹ÓÃ£¬Î´¾­×÷ÕßÐí¿É£¬²»µÃÓÃÓÚÆäËüÈÎºÎÓÃÍ¾
//ÖÐ¾°Ô°µç×Ó
//µêÆÌµØÖ·£ºhttp://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  ÎÄ ¼þ Ãû   : LED.C
//  °æ ±¾ ºÅ   : v2.0
//  ×÷    Õß   : HuangKai
//  Éú³ÉÈÕÆÚ   : 2014-0101
//  ×î½üÐÞ¸Ä   : 
//  ¹¦ÄÜÃèÊö   : ×Ö¿â°æ OLED SPI½Ó¿ÚÑÝÊ¾Àý³Ì(STM32ÏµÁÐ)
//              ËµÃ÷: 
//              ------------------------ÒÔÏÂÎªOLEDÏÔÊ¾ËùÓÃµ½µÄ½Ó¿----------------------------------------
//              GND    µçÔ´µØ
//              VCC  ½Ó5V»ò3.3vµçÔ´
//              D0   PA2£¨CLK£©
//              D1   PA3£¨DIN£©
//              RES  PA4
//              DC   PA5
//              CS   PA1                
//              --------------------------ÒÔÉÏÎªOLEDÏÔÊ¾ËùÓÃµ½µÄ½Ó¿Ú--------------------------------------
//              ------------------------ÒÔÏÂÎªOLED×Ö¿â²¿·ÖËùÓÃµ½µÄ½Ó¿----------------------------------------
//              OUT   PB10	SPI	
//              IN  	PB11
//              SCK   PB9
//              CS2   PB7                
//              --------------------------ÒÔÉÏÎªOLED×Ö¿â²¿·ÖËùÓÃµ½µÄ½Ó¿Ú--------------------------------------
// ÐÞ¸ÄÀúÊ·   :
// ÈÕ    ÆÚ   : 
// ×÷    Õß   : HuangKai
// ÐÞ¸ÄÄÚÈÝ   : ´´½¨ÎÄ¼þ
//°æÈ¨ËùÓÐ£¬µÁ°æ±Ø¾¿¡£
//Copyright(C) ÖÐ¾°Ô°µç×Ó2014/3/16
//All rights reserved
//******************************************************************************/
#include "led.h"

/*
 * º¯ÊýÃû£ºLED_GPIO_Config
 * ÃèÊö  £ºÅäÖÃLEDÓÃµ½µÄI/O¿Ú
 * ÊäÈë  £ºÎÞ
 * Êä³ö  £ºÎÞ
 */
void LED_GPIO_Config(void)
{		
	/*¶¨ÒåÒ»¸öGPIO_InitTypeDefÀàÐÍµÄ½á¹¹Ìå*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*¿ªÆôGPIOCµÄÍâÉèÊ±ÖÓ*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 

	/*Ñ¡ÔñÒª¿ØÖÆµÄGPIOCÒý½Å*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;	

	/*ÉèÖÃÒý½ÅÄ£Ê½ÎªÍ¨ÓÃÍÆÍìÊä³ö*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*ÉèÖÃÒý½ÅËÙÂÊÎª50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*µ÷ÓÃ¿âº¯Êý£¬³õÊ¼»¯GPIOC*/
  	GPIO_Init(GPIOB, &GPIO_InitStructure);		  

	/* ¹Ø±ÕËùÓÐledµÆ	*/
	GPIO_SetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2);	 
}


/******************* 2013                          ÐÅÒâµç×Ó¿Æ¼¼ *****END OF FILE************/
