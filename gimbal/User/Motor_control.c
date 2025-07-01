/* #include "Motor_control.h"
 extern pid_struct_t pid_motor_angle;
 extern rc_info_t rc_ctrl;
void Motor_ControlTask(void const * argument)
//{
    pid_init(&GM6020.Speed_pid,60,7,2,20000,30000);
	  pid_init(&GM6020.Angle_pid,0.25,0.05,0,1000,2500);
//	  float angle=0.0f;
	GM6020.Set_angle = 0.0f;
    while(1)
    {
//		angle+=(float)3.14/300;
//        if(angle > 6.28f)
//        {
//            angle = 0.0f;
//        }
//        GM6020.Set_angle = 4096.0f+(sin(angle) * 3800.0f);
//    GM6020.Set_angle = (rc_ctrl.rc.ch0/660.0f+1)*4096.0f;
	      GM6020.Set_angle +=	rc_ctrl.rc.ch0/16.0f;
			if(GM6020.Set_angle > 8191){
				GM6020.Set_angle = 0;}
			else if(GM6020.Set_angle < 0){
				GM6020.Set_angle = 8191;}
    PID_Calc_Angle(&GM6020.Angle_pid,GM6020.Set_angle,GM6020.rotor_angle,8192,95);
    GM6020.Set_Speed = GM6020.Angle_pid.output;
//		GM6020.Set_Speed = 70;
    PID_Calc_Speed(&GM6020.Speed_pid,GM6020.Set_Speed,GM6020.rotor_speed);
		Set_GM6020_Gimbal_Voltage(&hcan1,GM6020);
		VOFA_Tx();
		osDelay(1);
    }
}
 */
