#ifndef _delay_h_
#define _delay_h_

/**********************************************************
                     外部函数头文件                        
**********************************************************/

//#include "sys.h"
#include "stm32f1xx_hal.h"

void delay_us(uint32_t nus);     //延时n个us
void delay_ms(uint16_t nms);     //延时n个ms

#endif
