#include "VOFT_Uartx.h"

extern uint8_t VOFT_Data[20];
extern float Set_Yaw,Set_Pitch;
extern float IMU_angle[3];
extern Moto_GM6020_t GM6020;

#define TX_BUFFER_SIZE 32
uint8_t txBuffer[TX_BUFFER_SIZE];  // 发送缓冲区
volatile uint16_t txLength = 0;    // 当前缓冲区数据长度

int fputc(int ch, FILE *f)
{
    if(txLength < TX_BUFFER_SIZE-1)
	{
        txBuffer[txLength++] = (uint8_t)ch;
	}
	if(ch == '\n')
	{
		HAL_UART_Transmit_DMA(&huart1,txBuffer,txLength);
		txLength=0;
	}
    return ch;
}


void VOFA_Tx(void) {
    printf("%d,%.3f\r\n",
        GM6020.rotor_angle,
        (double)GM6020.Set_Angle);
}
