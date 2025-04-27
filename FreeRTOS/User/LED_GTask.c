#include "LED_GTask.h"
#include "Remote_Control.h"
#include "freertos.h"
osEvent event;
uint16_t received_ch0;
extern osMessageQId RCqueueHandle;
/**
* @brief 遥控器运动LED指数函数
*
* @note 接收消息队列中的rc数据并且在右摇杆向上的情况下亮灯
*/
void LED_GTask(void const * argument)
{
    while (1)
    {
        //HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
			   event = osMessageGet(RCqueueHandle, osWaitForever);

        if (event.status == osEventMessage) {  // 检查是否为有效数据
            received_ch0 = (uint16_t)event.value.v;  // 提取数据，注意要转换回uint16格式

            if (received_ch0 > 1024) {
                
                HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
            }
        }
        osDelay(500);
    }
}


