#include "dri_uart.h"
#include "pid.h"
extern Moto_GM6020_t GM6020;

#define TX_BUFFER_SIZE 256
uint8_t txBuffer[TX_BUFFER_SIZE];  // 发送缓冲区
volatile uint16_t txLength = 0;    // 当前缓冲区数据长度
//uint8_t test=0;
int fputc(int ch, FILE *f)
{
    if(txLength < TX_BUFFER_SIZE-1)
	{
        txBuffer[txLength++] = (uint8_t)ch;
	}
	if(ch == '\n')
	{
		HAL_UART_Transmit_DMA(&huart6,txBuffer,txLength);
		//test++;
		txLength=0;
	}
    return ch;
}

void VOFA_Tx(void) {
    printf("%d,%d,%f,%f,%d\r\n",GM6020.rotor_angle,GM6020.rotor_speed,GM6020.Set_Angle,GM6020.Set_Speed,GM6020.temp);
}

