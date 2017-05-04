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