#include "task_cpp.h"

#include "cancan.h"
DM4310 motor;
void dm_task(void const * argument)
{
    CAN_Filter_Init();
    osDelay(1000);
    motor.Bind_CAN(&hcan1);
    motor.Set_Can_ID(0x01);
    motor.Set_Master_ID(0x00);
    motor.Set_CTL_Mode(MIT_MODE);
    motor.Enable();
    motor.Set_Speed(5);
    HAL_GPIO_WritePin(GPIOH , GPIO_PIN_10,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH , GPIO_PIN_11,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOH , GPIO_PIN_12,GPIO_PIN_SET);
    while (1)
    {

        osDelay(1000);
    }
}