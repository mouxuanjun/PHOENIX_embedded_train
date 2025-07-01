#include "dri_uart.h"
#include "usbd_cdc_if.h"
extern Moto_GM6020_t motor_pitch;  //0x205
extern Moto_GM6020_t motor_yaw;    //0x207
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
		CDC_Transmit_FS(txBuffer, txLength);
		//HAL_UART_Transmit_DMA(&huart1,txBuffer,txLength);
		txLength=0;
	}
    return ch;
}

//void VOFA_Tx(void) 
//{
//    printf("%d,%.3f\r\n",
//            GM6020.rotor_angle,
//    (double)GM6020.Set_angle
//	         );
//}

