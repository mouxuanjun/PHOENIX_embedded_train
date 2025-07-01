#include "Motor_control.h"
extern pid_struct_t pid_motor_angle;
extern pid_struct_t pid_motor_speed;
extern rc_info_t rc_ctrl;
void Motor_ControlTask(void const * argument)
{
	
      pid_init(&motor_pitch.Speed_pid,80,2.45,0, 0,1000,18000);
	    pid_init(&motor_yaw.Speed_pid,380,2,0.4,1000,1000,16800);//(250,10,0)
	  //motor_yaw.Set_Speed = 10;
   pid_init(&motor_yaw.Angle_pid,0.25,0,0.9,0,1000,2500);//(1,0,0)
	    pid_init(&motor_pitch.Angle_pid,0.67,0,1,0,1000,2500);//(0.7, 0.001,0.75)
//	  float angle=0.0f;
	    motor_yaw.Set_angle =0.0f;
      motor_pitch.Set_angle =1600.0f ;
	while(1)
      {
				switch(rc_ctrl.rc.s2)
		{
		case 3:
	{
//	    	angle+=(float)3.14/300;
//        if(angle > 6.28f)
//        {
//            angle = 0.0f;
//        }
//       motor_pitch.Set_angle = 4096.0f+(sin(angle) * 3800.0f);
        motor_yaw.Set_angle -=	rc_ctrl.rc.ch0/80.0f;
			motor_pitch.Set_angle +=	rc_ctrl.rc.ch3/160.0f;
		
		  	if(motor_yaw.Set_angle > 8191){
				motor_yaw.Set_angle = 0;}
		  	else if(motor_yaw.Set_angle < 0){
				motor_yaw.Set_angle = 8191;}
				
				if(motor_pitch.Set_angle > 2300){
				motor_pitch.Set_angle = 2300;}
		  	else if(motor_pitch.Set_angle < 900){
				motor_pitch.Set_angle = 900;}
				
				PID_Calc_Angle(&motor_yaw.Angle_pid,motor_yaw.Set_angle,motor_yaw.rotor_angle,8192,100);
				PID_Calc_Angle(&motor_pitch.Angle_pid,motor_pitch.Set_angle,motor_pitch.rotor_angle,8192,100);
      motor_yaw.Set_Speed = motor_yaw.Angle_pid.output;
				motor_pitch.Set_Speed = motor_pitch.Angle_pid.output;
        PID_Calc_Speed(&motor_pitch.Speed_pid,motor_pitch.Set_Speed,motor_pitch.rotor_speed);
				PID_Calc_Speed(&motor_yaw.Speed_pid,motor_yaw.Set_Speed,motor_yaw.rotor_speed);
		    Set_GM6020_Gimbal_yaw_Voltage(&hcan1,motor_yaw);
				Set_GM6020_Gimbal_pitch_Voltage(&hcan2,motor_pitch);
//		   VOFA_Tx();
	   	 osDelay(1);}
	case 1:
{
//	pid_init(&motor_pitch.Speed_pid,0,0,0,0,0);
//	    pid_init(&motor_yaw.Speed_pid,0,0,0,0,0);
////	  motor_yaw.Set_Speed = 10;
//	    pid_init(&motor_yaw.Angle_pid,0,0,0,0,0);
//	    pid_init(&motor_pitch.Angle_pid,0,0,0,0,0);
}
		}
	}		
}

