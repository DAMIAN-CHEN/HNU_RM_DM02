#include "ws2812.h"

#define WS2812_LowLevel    0xC0     // 0
#define WS2812_HighLevel   0xF0     // 1

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
