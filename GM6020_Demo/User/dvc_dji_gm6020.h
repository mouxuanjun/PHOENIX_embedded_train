#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"
#include "can.h"
#include "stm32f4xx.h"


typedef struct
{
    uint16_t can_id;//µç»úID
    int16_t  set_voltage;//Éè¶¨µÄµçÑ¹Öµ
    float Set_Speed;//Éè¶¨ËÙ¶È
    float Set_Angle;//Éè¶¨½Ç¶È

    uint16_t rotor_angle;//µç»ú½Ç¶È
    int16_t  rotor_speed;//µç»úËÙ¶È
    int16_t  torque_current;//µç»úÅ¤¾Ø
    uint8_t  temp;//ÎÂ¶È

}Moto_GM6020_t;

/*typedef struct
{
		uint16_t StdId; //æ ‡è¯†ç¬¦
    uint8_t IDE; //
		uint8_t RTR; //
		uint8_t DLC;//DLC
}chassis_tx_message;*/

void Set_GM6020_Gimbal_Voltage(CAN_HandleTypeDef* hcan,Moto_GM6020_t GM6020_Yaw,Moto_GM6020_t GM6020_Pitch);
void Get_GM6020_Motor_Message(uint32_t StdId,uint8_t rx_data[8]);
void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
#endif
