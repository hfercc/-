void RCC_Configuration(void)                 //Ê¹ÓÃÈÎºÎÒ»¸öÍâÉèÊ±£¬Îñ±Ø¿ªÆôÆäÏàÓ¦µÄÊ±ÖÓ
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
}


void GPIO_Configuration(void)            //Ê¹ÓÃÄ³io¿ÚÊäÈëÊä³öÊ±£¬ÇëÎñ±Ø¶ÔÆä³õÊ¼»¯ÅäÖÃ
{
    GPIO_InitTypeDef GPIO_InitStructure;   //¶¨Òå¸ñÊ½ÎªGPIO_InitTypeDefµÄ½á¹¹ÌåµÄÃû×ÖÎªGPIO_InitStructure
                                          //typedef struct { u16 GPIO_Pin; GPIOSpeed_TypeDef GPIO_Speed; GPIOMode_TypeDef GPIO_Mode; } GPIO_InitTypeDef;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //ÅäÖÃIO¿Ú¹¤×÷Ä£Ê½Îª ÍÆÍìÊä³ö£¨ÓÐ½ÏÇ¿µÄÊä³öÄÜÁ¦£©
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //ÅäÖÃIO¿Ú×î¸ßµÄÊä³öËÙÂÊÎª50M
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_Init(GPIOC, &GPIO_InitStructure);        //³õÊ¼»¯GPIOCµÄÏàÓ¦IO¿ÚÎªÉÏÊöÅäÖÃ£¬ÓÃÓÚledÊä³ö

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


}
