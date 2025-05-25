#include "Chassis.h"

void Chassis_task(void const * argument){
    CHASSIS_InitArgument();
    while(1)
    {
        if(SystemValue == Starting)
        {
            SystemValue = Running;
        } else {
            if(rc_ctrl.s2==1)
                Chassis_way=Chassis_follow_gimbal;
            else if(rc_ctrl.s2==3)
                Chassis_way=Chassis_normal;
            else if(rc_ctrl.s2==2)
                Chassis_way=Chassis_gyrescope;
            Control();
            CHASSIS_Single_Loop_Out();
        }
    }
}
