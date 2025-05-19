#include "Motor_control.h"

void Motor_ControlTask(void const * argument)
{
    PID_init(&GM6020.Speed_PID,20,0.5,0,0,10000,25000);//20,0.5,0,0//20,0.5,0,0
	PID_init(&GM6020.Angle_PID,0,0,0,0,1000,2500);//Ç°À¡£º20//2.8,0,2,0
	float angle=0.0f;
    while(1)
    {
//		angle+=(float)3.14/300;
//        if(angle > 6.28f)
//        {
//            angle = 0.0f;
//        }
//        GM6020.Set_Angle = 4096.0f+(sin(angle) * 3800.0f);
//        PID_Calc_Angle(&GM6020.Angle_PID,GM6020.Set_Angle,GM6020.rotor_angle,8192,100);
        GM6020.Set_Speed = GM6020.Angle_PID.output;
        PID_Calc_Speed(&GM6020.Speed_PID,GM6020.Set_Speed,GM6020.rotor_speed);
		Set_GM6020_Gimbal_Voltage(&hcan1,GM6020);
		VOFA_Tx();
		osDelay(1);
    }
}
