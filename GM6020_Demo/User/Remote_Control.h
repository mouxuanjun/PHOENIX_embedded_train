#ifndef __Remote_Control_H__
#define __Remote_Control_H__

#include "main.h"
#include "freertos.h"
#include "cmsis_os.h"


typedef __packed struct 
{ 
__packed struct 
{ 
uint16_t ch0; 
uint16_t ch1; 
uint16_t ch2; 
uint16_t ch3; 
uint8_t  sL; 
uint8_t  sR; 
}rc; 
__packed struct 
{ 
int16_t x; 
int16_t y; 
int16_t z; 
uint8_t press_l; 
uint8_t press_r; 
}mouse; 
__packed struct 
{ 
uint16_t v; 
}key; 
}RC_Ctl_t; 

void RC_init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
#endif
