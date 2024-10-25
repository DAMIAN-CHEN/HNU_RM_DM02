#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0��
#define WS2812_HighLevel   0xF0     // 1��

 static uint8_t red=0;
 static uint8_t green=0;
 static uint8_t blue=0;
 static float time_scale=0;
 static int rgb_flag=0;
 static uint8_t lighting=0;
 
enum color
{
	RED_H=0,
	GREEN_H=1,
	BLUE_H=2,
	RED_L=3,
	GREEN_L=4,
	BLUE_L=5,	
};

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;
    for (int i = 0; i < 8; i++)
    {
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}

void led_blinky_a()
{
if(rgb_flag==RED_H)
{
	WS2812_Ctrl(lighting, 0, 0);
	lighting++;
	if(lighting>=200)rgb_flag=RED_L;	
}
else if(rgb_flag==RED_L)
{
	WS2812_Ctrl(lighting,0, 0);
	lighting--;
	if(lighting<=3)rgb_flag=BLUE_H;
}
else if(rgb_flag==BLUE_H)
{
	WS2812_Ctrl(0, lighting, 0);
	lighting++;
	if(lighting>=200)rgb_flag=BLUE_L;
}
else if(rgb_flag==BLUE_L)
{
	WS2812_Ctrl(0, lighting, 0);
	lighting--;
	if(lighting<=3)rgb_flag=GREEN_H;
}	
else if(rgb_flag==GREEN_H)
{
	WS2812_Ctrl(0, 0, lighting);
	lighting++;
	if(lighting>=200)rgb_flag=GREEN_L;
}	
else if(rgb_flag==GREEN_L)
{
	WS2812_Ctrl(0, 0, lighting);
	lighting--;
	if(lighting<=3)rgb_flag=RED_H;
}	
}

void led_blinky_boat(float servo_red,float motor_blue)
{
	red=10000*(servo_red-0.065);
	blue=10000*(motor_blue-0.065);
	
	red=50+50*cos(6*time_scale);
	blue=50+50*sin(2*time_scale);
	green=50+50*sin(3*time_scale);
	time_scale+=0.0015;
	
	WS2812_Ctrl(red , green , blue);
	if(time_scale>=1000)time_scale=1;
}