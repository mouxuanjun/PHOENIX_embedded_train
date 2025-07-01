#include "dri_uart.h"
#include "usbd_cdc_if.h"
#define TX_BUFFER_SIZE 256
uint8_t txBuffer[TX_BUFFER_SIZE];
volatile uint16_t txLength = 0;

int fputc(int ch, FILE *f)
{
    if(txLength < TX_BUFFER_SIZE-1)
	{
        txBuffer[txLength++] = (uint8_t)ch;
	}
	if(ch == '\n')
	{
		//CDC_Transmit_FS(txBuffer, txLength);
		HAL_UART_Transmit_DMA(&huart1,txBuffer,txLength);
		txLength=0;
	}
    return ch;
}

/* void VOFA_Tx(void) 
{
    printf("%f,%.3f\r\n",
            DM4310.rotor_angle,
    (double)DM4310.Set_angle
	         );
} */

