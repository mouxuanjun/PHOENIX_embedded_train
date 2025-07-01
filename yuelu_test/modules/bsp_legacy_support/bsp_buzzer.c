#include "bsp_buzzer.h"
#include "main.h"

#warning this is a legacy support file, please use the new version

extern TIM_HandleTypeDef htim4;
static uint8_t tmp_warning_level = 0;

void BuzzerInit()
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
}

void BuzzerOn( )
{
    static int16_t temp = 4000 ;
    if(temp < 1000)
    {    
        BuzzerOff();
        return;
    }
    __HAL_TIM_PRESCALER(&htim4,(int)(temp/1000));
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 10000);
    temp -= 16 ;
    
}

void BuzzerOff(void)
{
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
    tmp_warning_level = 0;
}
