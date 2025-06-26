#include "GM6020.h"

extern Moto_GM6020_t GM6020;
extern Moto_GM6020_t GM6020_pitch;

/**
 * @file GM6020.c
 * @brief GM6020½ÓÊÜ·´À¡±¨ÎÄº¯Êý
 * @param StdId µç»úID
 * @param rx_data CANÍ¨µÀÊÕµ½µÄÊý¾Ý
 * @author HWX
 * @date 2024/10/20
 */
void Get_GM6020_Motor_Message(uint32_t StdId,uint8_t rx_data[8])
{
    switch(StdId)//½ÓÊÕÖ¸¶¨µç»ú·´À¡µÄÐÅÏ¢
    {
        case 0x207://·´À¡±¨ÎÄ±êÊ¶·û
        {
            GM6020.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//½ÓÊÕ»úÐµ½Ç¶È£¨16bit£©
            GM6020.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//½ÓÊÕ×ªËÙ£¨16bit£©
            GM6020.torque_current = ((rx_data[4] << 8) | rx_data[5]);//½ÓÊÕÊµ¼Ê×ª¾Ø
            GM6020.temp           =   rx_data[6];//½ÓÊÕµç»úÎÂ¶È£¨8bit£©
            break;
        }
				case 0x205://Â·Â´Ã€Â¡Â±Â¨Î„Â±êŠ¶Â·Ã»
        {
            GM6020_pitch.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//Â½ÓŠÕ»ÃºÐµÂ½Ç¶È£Â¨16bitÂ£Â©
            GM6020_pitch.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//Â½ÓŠÕ—ÂªË™Â£Â¨16bitÂ£Â©
            GM6020_pitch.torque_current = ((rx_data[4] << 8) | rx_data[5]);//Â½ÓŠÕŠÂµÂ¼Ê—ÂªÂ¾Ø
            GM6020_pitch.temp           =   rx_data[6];//Â½ÓŠÕµç»ºÎ‚Â¶È£Â¨8bitÂ£Â©
            break;
        }
    }
}

void Set_GM6020_Gimbal_Voltage(CAN_HandleTypeDef* hcan,Moto_GM6020_t GM6020)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t             CAN1_tx_data[8] = {0};
    
    tx_header.StdId = 0X200;//±êÊ¶·û£¨¼ûÊÖ²áP6£©
    tx_header.IDE   = CAN_ID_STD;//±ê×¼ID
    tx_header.RTR   = CAN_RTR_DATA;//Êý¾ÝÖ¡
    tx_header.DLC   = 8;//×Ö½Ú³¤¶È
    CAN1_tx_data[2] = ((int16_t)GM6020.Speed_PID.output>>8)&0xff;
    CAN1_tx_data[3] = ((int16_t)GM6020.Speed_PID.output)&0xff;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, CAN1_tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}
