#include "Motor_control.h"
#include "Motor4310.h"
#include "Remote_Control.h"
#include "LK9025.h"
#define MAX_MOTOR 3
extern RC_Ctl_t ctl;
uint8_t sbus_buf[18];
extern LKMotor_Measure_t measure;
int16_t lk_controls[4] = {0x00, 0x00, 0x00, 0x00};
void Motor_ControlTask(void const * argument)
{
	//速度
  PID_init(&GM6020.Speed_PID,450,5,3,0,10000,25000);
	PID_init(&GM6020_pitch.Speed_PID,93,5.1,2.3,0,10000,25000);
	//角度
	PID_init(&GM6020.Angle_PID,0.25,0,0.1,0,1000,2500);
	PID_init(&GM6020_pitch.Angle_PID,0.15,0,5.300,0,1000,2500);
	//4310
	PID_init(&DM4310.Speed_PID,0,0,0,0,10000,25000);
	PID_init(&DM4310.Angle_PID,0,0,0,0,1000,2500);
	//MS9025
	PID_init(&MS9025.Speed_PID,0,0,0,0,10000,25000);
	PID_init(&MS9025.Angle_PID,0,0,0,0,1000,2500);
	// float angle=0.0f;  // 暂时注释掉未使用的变量
	//初始化9025电机
	Motor_LK9025_Enable(&hcan1, 0x141); // 启用9025电机
	Motor_LK9025_Enable(&hcan1, 0x142); // 启用9025电机
		Motor_LK9025_Enable(&hcan2, 0x141); // 启用9025电机

	Motor_LK9025_Enable(&hcan2, 0x142); // 启用9025电机
	//初始化DM4310电机
//	dm4310_motor_init();
//	for (int i=1;i<MAX_MOTOR;i++){
//	cur_dm4310(i);
//	ctrl_enable();
//	}
    while(1)
    {
			if(ctl.rc.sL!=1){
//		angle+=(float)3.14/300;
//        if(angle > 6.28f)
//        {
//            angle = 0.0f;
//        }
//        GM6020.Set_Angle = 4096.0f+(sin(angle) * 3800.0f);
		GM6020.Set_Angle-=(ctl.rc.ch0-1024)*0.01;
				
		if(GM6020.Set_Angle>=8192){
			GM6020.Set_Angle=0;
		}
		if(GM6020.Set_Angle<0){
			GM6020.Set_Angle+=8191;
		}
			
        PID_Calc_Angle(&GM6020.Angle_PID,GM6020.Set_Angle,GM6020.rotor_angle,8192,100);
        GM6020.Set_Speed = GM6020.Angle_PID.output;
        PID_Calc_Speed(&GM6020.Speed_PID,GM6020.Set_Speed,GM6020.rotor_speed);
		//Set_GM6020_Gimbal_Voltage(&hcan1,GM6020);
			  int16_t motor_command=GM6020.Speed_PID.output;
				
				
				
		GM6020_pitch.Set_Angle-=(ctl.rc.ch1-1024)*0.01;
				
		if(GM6020_pitch.Set_Angle>=8192){
			GM6020_pitch.Set_Angle=0;
		}
		if(GM6020_pitch.Set_Angle<0){
			GM6020_pitch.Set_Angle+=8191;
		}
	
		
		if(GM6020_pitch.Set_Angle>=2300)GM6020_pitch.Set_Angle=2300;
		if(GM6020_pitch.Set_Angle<=1100)GM6020_pitch.Set_Angle=1100;
		PID_Calc_Angle(&GM6020_pitch.Angle_PID,GM6020_pitch.Set_Angle,GM6020_pitch.rotor_angle,8192,150);
		GM6020_pitch.Set_Speed = GM6020_pitch.Angle_PID.output;
		PID_Calc_Speed(&GM6020_pitch.Speed_PID,GM6020_pitch.Set_Speed,GM6020_pitch.rotor_speed);
		//int16_t motor_command2=0x00;
		int16_t motor_command2=GM6020_pitch.Speed_PID.output;
		
				
			
			//Send_GM6020_Motor_Message(motor_command2, 0x00,motor_command, 0x00); 
			//这里控制MS9025


			//MS9025.Set_Angle = (ctl.rc.ch2-1024)*0.1f;
			PID_Calc_Angle(&MS9025.Angle_PID,MS9025.Set_Angle,measure.angle_single_round,16383,150);
			MS9025.Set_Speed = MS9025.Angle_PID.output;
		  //MS9025.Set_Speed = 0;	
			PID_Calc_Speed(&MS9025.Speed_PID,MS9025.Set_Speed,measure.speed_rads);
			if(MS9025.Speed_PID.output>2048)MS9025.Speed_PID.output=2048;
			if(MS9025.Speed_PID.output<-2048)MS9025.Speed_PID.output=-2048;
			//Motor_LK9025_control_speed(&hcan1,0x141,MS9025.Speed_PID.output);
			Motor_LK9025_control_torque(&hcan2, 0x142,MS9025.Speed_PID.output); 
			Motor_LK9025_control_torque(&hcan2, 0x141,MS9025.Speed_PID.output);
			Motor_LK9025_control_torque(&hcan1, 0x142,MS9025.Speed_PID.output); 
			Motor_LK9025_control_torque(&hcan1, 0x141,MS9025.Speed_PID.output);
			lk_controls[0]= MS9025.Speed_PID.output;
		  Motor_LK9025_Control_Torque_Multi(&hcan1,lk_controls);
			
		//VOFA_Tx();
			}else{
		Send_GM6020_Motor_Message(0x00, 0x00,0x00, 0x00);
		Motor_LK9025_disable(&hcan1,1);
			}
		osDelay(1);
			
    }
			
}
CAN_TxHeaderTypeDef Ctx;
uint8_t chassis_can_send_data[8];
void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box; 
    
    Ctx.StdId = CAN_CHASSIS_ALL_ID;
    Ctx.IDE = CAN_ID_STD;
    Ctx.RTR = CAN_RTR_DATA;
    Ctx.DLC = 0x08;
    
    chassis_can_send_data[0] = (uint8_t)(motor1 >> 8); 
    chassis_can_send_data[1] = (uint8_t)(motor1); 
    chassis_can_send_data[2] = (uint8_t)(motor2 >> 8); 
    chassis_can_send_data[3] = (uint8_t)(motor2); 
    chassis_can_send_data[4] = (uint8_t)(motor3 >> 8); 
    chassis_can_send_data[5] = (uint8_t)(motor3); 
    chassis_can_send_data[6] = (uint8_t)(motor4 >> 8); 
    chassis_can_send_data[7] = (uint8_t)(motor4); 
 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Ctx, chassis_can_send_data, &send_mail_box); 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN2, &Ctx, chassis_can_send_data, &send_mail_box); 
}