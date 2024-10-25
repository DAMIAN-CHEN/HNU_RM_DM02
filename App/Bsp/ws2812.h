#ifndef __WS2812_H__
#define __WS2812_H__
 
 
#include "main.h" 


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;
 
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
void led_blinky_a();
void led_blinky_boat(float servo_red,float motor_blue);
#endif
