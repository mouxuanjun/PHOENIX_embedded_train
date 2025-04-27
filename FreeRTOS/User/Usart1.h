#ifndef __USART1_H__
#define __USART1_H__

#include "main.h"
#include "freertos.h"
#include "cmsis_os.h"
void usart_Send(void);
void usart_Task2(void const * argument);
#endif
