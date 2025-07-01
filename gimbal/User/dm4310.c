#include "main.h"
#include "dm4310.h"
#include "cmsis_os.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

Motor_t MOTOR1_t,MOTOR2_t,MOTOR3_t;

/**
 * @brief
 * @param  x_int
 * @param  x_min
 * @param  bits
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

/**
 * @brief
 * @param  x_int
 * @param  x_min
 * @param  bits
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{ 
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

/**
 * @brief
 * @param  hcan
 * @param  ID
 * @param  _pos
 * @param  _vel
 * @param  _KP
 * @param  _KD
 * @param  _torq
 */
void MIT_CtrlMotor(CAN_HandleTypeDef* hcan,uint16_t ID, float _pos, float _vel,float _KP, float _KD, float _torq)
{
    static CAN_TxPacketTypeDef packet;
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    
    packet.hdr.StdId = 0x01;
    packet.hdr.IDE = CAN_ID_STD;
    packet.hdr.RTR = CAN_RTR_DATA;
    packet.hdr.DLC = 0x08;

    pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(_KP, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(_KD, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(_torq, T_MIN, T_MAX, 12);

    packet.tx_data[0] = (pos_tmp >> 8);
    packet.tx_data[1] = pos_tmp;
    packet.tx_data[2] = (vel_tmp >> 4);
    packet.tx_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    packet.tx_data[4] = kp_tmp;
    packet.tx_data[5] = (kd_tmp >> 4);
    packet.tx_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    packet.tx_data[7] = tor_tmp;

	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
}

/**
 * @brief
 * @param  hcan
 * @param  ID
 * @param  _pos
 * @param  _vel
 */
//void PosSpeed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _pos, float _vel)
//{
//	static CAN_TxPacketTypeDef packet;
//	uint8_t *pbuf,*vbuf;
//	pbuf=(uint8_t*)&_pos;
//	vbuf=(uint8_t*)&_vel;

//	packet.hdr.StdId = 0x02;
//	packet.hdr.IDE = CAN_ID_STD;
//	packet.hdr.RTR = CAN_RTR_DATA;
//	packet.hdr.DLC = 0x08;

//	packet.payload[0] = *pbuf;;
//	packet.payload[1] = *(pbuf+1);
//	packet.payload[2] = *(pbuf+2);
//	packet.payload[3] = *(pbuf+3);
//	packet.payload[4] = *vbuf;
//	packet.payload[5] = *(vbuf+1);
//	packet.payload[6] = *(vbuf+2);
//	packet.payload[7] = *(vbuf+3);

//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
//	{
//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//	{
//		HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}
//}


///**
//* @brief
//* @param  hcan
//* @param  ID
//* @param  _vel
//*/
//void Speed_CtrlMotor(CAN_HandleTypeDef* hcan, uint16_t ID, float _vel)
//{
//	static CAN_TxPacketTypeDef packet;
//	uint8_t *vbuf;
//	vbuf=(uint8_t*)&_vel;

//	packet.hdr.StdId = 0x02;
//	packet.hdr.IDE = CAN_ID_STD;
//	packet.hdr.RTR = CAN_RTR_DATA;
//	packet.hdr.DLC = 0x04;

//	packet.payload[0] = *vbuf;
//	packet.payload[1] = *(vbuf+1);
//	packet.payload[2] = *(vbuf+2);
//	packet.payload[3] = *(vbuf+3);

//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
//	{
//	if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//	{
//		HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.payload, (uint32_t*)CAN_TX_MAILBOX2);
//	}
//	}
//}

/**
 * @brief 
 * @param  hcan  
 * @param  ID    
 * @param  data 
 */
 void Enable_CtrlMotor(CAN_HandleTypeDef* hcan,uint8_t ID, uint8_t data0, uint8_t data1,uint8_t data2, uint8_t data3, uint8_t data4,uint8_t data5,uint8_t data6,uint8_t data7)
 {
    static CAN_TxPacketTypeDef packet;
    switch(ID)
    {
        case 0x01:
        {
            packet.hdr.StdId = 0x01;
            packet.hdr.IDE = CAN_ID_STD;
            packet.hdr.RTR = CAN_RTR_DATA;
            packet.hdr.DLC = 0x08;
            packet.tx_data[0] = (uint8_t)data0;
            packet.tx_data[1] = (uint8_t)data1;
            packet.tx_data[2] = (uint8_t)data2;
            packet.tx_data[3] = (uint8_t)data3;
            packet.tx_data[4] = (uint8_t)data4;
            packet.tx_data[5] = (uint8_t)data5;
            packet.tx_data[6] = (uint8_t)data6;
    packet.tx_data[7] = (uint8_t)data7;

	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
  }
  case 0x02:
        {
            packet.hdr.StdId = 0x02;
            packet.hdr.IDE = CAN_ID_STD;
            packet.hdr.RTR = CAN_RTR_DATA;
            packet.hdr.DLC = 0x08;
            packet.tx_data[0] = (uint8_t)data0;
            packet.tx_data[1] = (uint8_t)data1;
            packet.tx_data[2] = (uint8_t)data2;
            packet.tx_data[3] = (uint8_t)data3;
            packet.tx_data[4] = (uint8_t)data4;
            packet.tx_data[5] = (uint8_t)data5;
            packet.tx_data[6] = (uint8_t)data6;
    packet.tx_data[7] = (uint8_t)data7;

	  if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK) //
	  {
		if(HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
		{
			HAL_CAN_AddTxMessage(hcan, &packet.hdr, packet.tx_data, (uint32_t*)CAN_TX_MAILBOX2);
    }
    }
  }
}
}
/**
 * @brief
 * @param  canHandle
 */
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *canHandle)
{
	static CAN_RxPacketTypeDef packet;
	HAL_CAN_GetRxMessage(canHandle, CAN_RX_FIFO0, &packet.hdr, packet.rx_data);
    if (canHandle->Instance == CAN1)
    {         MOTOR1_t.error=packet.rx_data[0]>>4;
              MOTOR1_t.p_int=((packet.rx_data[1]<<8)|packet.rx_data[2]);
              MOTOR1_t.v_int=(packet.rx_data[3]<<4)|(packet.rx_data[4]>>4);
              MOTOR1_t.t_int=((packet.rx_data[4]&0xF)<<8)|packet.rx_data[5];
              MOTOR1_t.position = uint_to_float(MOTOR1_t.p_int, P_MIN, P_MAX, 16); 
              MOTOR1_t.velocity = uint_to_float(MOTOR1_t.v_int, V_MIN, V_MAX, 12);
              MOTOR1_t.torque = uint_to_float(MOTOR1_t.t_int, T_MIN, T_MAX, 12); 
        if(MOTOR1_t.position > 6.25f){
					MOTOR1_t.position -= 6.25f;
    }
				else if(MOTOR1_t.position < 0)
				{MOTOR1_t.position += 12.5f;
					if(MOTOR1_t.position > 6.25f){
					MOTOR1_t.position -= 6.25f;
    }
				}
			}	
      else if (canHandle->Instance == CAN2)
      {
        MOTOR2_t.error=packet.rx_data[0]>>4;
        MOTOR2_t.p_int=((packet.rx_data[1]<<8)|packet.rx_data[2]);
        MOTOR2_t.v_int=(packet.rx_data[3]<<4)|(packet.rx_data[4]>>4);
        MOTOR2_t.t_int=((packet.rx_data[4]&0xF)<<8)|packet.rx_data[5];
        MOTOR2_t.position = uint_to_float(MOTOR2_t.p_int, P_MIN, P_MAX, 16); 
        MOTOR2_t.velocity = uint_to_float(MOTOR2_t.v_int, V_MIN, V_MAX, 12);
        MOTOR2_t.torque = uint_to_float(MOTOR2_t.t_int, T_MIN, T_MAX, 12); 
      
       if(MOTOR2_t.position > 6.25f){
					MOTOR2_t.position -= 6.25f;
    }
				else if(MOTOR2_t.position < 0)
				{MOTOR2_t.position += 12.5f;
					if(MOTOR2_t.position > 6.25f){
					MOTOR2_t.position -= 6.25f;
    }
				}
      }
		HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
}
void MIT_task(void const * argument)
{
	    
//	    pid_init(&MOTOR1_t.Speed_pid,1.258,0.00525,3.2315,18,18);
	    pid_init(&MOTOR1_t.Speed_pid,0.5,0.01,0.5,18,18);
//	    MOTOR1_t.Set_Speed = 10.0f;
	    pid_init(&MOTOR1_t.Angle_pid,120,0,40,1000,2500);
//	float angle=0.0f;
	MOTOR1_t.Set_angle = 0.0f;
	while(1)
      {
     MOTOR1_t.Set_angle +=	rc_ctrl.rc.ch0/26500.0f;
		  	if(MOTOR1_t.Set_angle > 6.25f){
				MOTOR1_t.Set_angle = 0;}
		  	else if(MOTOR1_t.Set_angle < 0){
				MOTOR1_t.Set_angle = 6.25f;}
//			    	angle+=(float)3.14/300;
//        if(angle > 6.28f)
//        {
//            angle = 0.0f;
//        }
//        MOTOR1_t.Set_angle = 4096.0f+(sin(angle) * 3800.0f);
       PID_Calc_Angle(&MOTOR1_t.Angle_pid,MOTOR1_t.Set_angle,MOTOR1_t.position,6.25f,100);
        MOTOR1_t.Set_Speed = MOTOR1_t.Angle_pid.output;
				PID_Calc_Speed(&MOTOR1_t.Speed_pid,MOTOR1_t.Set_Speed,MOTOR1_t.velocity);
        MIT_CtrlMotor(&hcan1,MOTOR1,0,0,0,0,MOTOR1_t.Speed_pid.output);
   	    osDelay(1);
			}
}	
