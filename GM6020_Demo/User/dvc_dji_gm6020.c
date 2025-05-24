#include "dvc_dji_gm6020.h"
#include "dri_can.h"
HAL_StatusTypeDef can_1;
#define Motor_1_ID 0x207
#define Motor_2_ID 0x205
extern Moto_GM6020_t GM6020;
extern Moto_GM6020_t GM6020_pitch;
#define CAN_CHASSIS_ALL_ID 0x1FF           //标识符的id
#define CHASSIS_CAN hcan1
#define CHASSIS_CAN2 hcan2
uint8_t chassis_can_send_data[8];
//uint8_t test2=0;
float last_value;
/**
 * @file GM6020.c
 * @brief GM6020���ܷ������ĺ���
 * @param StdId ���ID
 * @param rx_data CANͨ���յ�������
 * @author HWX
 * @editor CGH
 * @date 2025/5/16
 */
 volatile uint32_t temp3;
 /**
 * @brief 限幅滤波函数（改进版）
 * @param[in] new_val 本次采集的原始传感器值
 * @param[in,out] last_val 指向上次有效值的指针（函数内部会更新）
 * @param[in] max_step 允许的最大单步变化量（默认值500.0）
 * @param[in] smooth_factor 平滑系数（0.0~1.0，0表示无平滑）
 * @return float 处理后的有效值
 * @note 
 * - 当变化量超过max_step时，输出值会渐变过渡
 * - 建议初始化时设置合理的last_val初始值
 * @warning 禁止在多线程环境不加锁直接使用
 * @example
 * float last = 0.0f;
 * float filtered = advanced_filter(600.0f, &last, 500.0f, 0.2f); 
 * // 输出last将变为500.0
 */
float filter(float new_val, float last_val) {
    // 使用绝对值比较双向突变
    if(new_val - last_val > 1000.0f||new_val-last_val< -1000.0f) { 
        // 返回上次值作为本次有效值
        return last_val;
    }
    // 正常情况返回当前值
    return new_val;
}


void Get_GM6020_Motor_Message(uint32_t StdId,uint8_t rx_data[8])
{
		temp3 = StdId;
    switch(StdId)//����ָ�������������Ϣ
    {
        case 0x207://�������ı�ʶ��
        {
            //test2++;
					  static float last_value=-1;
						static float last_speed=-1;
					  if(last_value>=0)last_value=GM6020.rotor_angle;
						if(last_value>=0)last_speed=GM6020.rotor_speed;
 					  GM6020.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//���ջ�е�Ƕȣ�16bit��
            GM6020.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//����ת�٣�16bit��
            GM6020.torque_current = ((rx_data[4] << 8) | rx_data[5]);//����ʵ��ת��
            GM6020.temp           =   rx_data[6];//���յ���¶ȣ�8bit��
					  if(last_value<0){
							last_value=GM6020.rotor_angle;
							last_speed=GM6020.rotor_speed;
						}
						GM6020.rotor_angle=filter(GM6020.rotor_angle,last_value);
						GM6020.rotor_speed=filter(GM6020.rotor_speed,last_speed);
            break;
        }
				case 0x205:
        {
            //test2++;
 					  GM6020_pitch.rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);//½ӊջúе½Ƕȣ¨16bit£©(神秘乱码）
            GM6020_pitch.rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);//½ӊ՗ª˙£¨16bit£©
            GM6020_pitch.torque_current = ((rx_data[4] << 8) | rx_data[5]);//½ӊՊµ¼ʗª¾؍
            GM6020_pitch.temp           =   rx_data[6];//½ӊյ绺΂¶ȣ¨8bit£©
            break;
        }
				default:
				{
				break;
				}
				
					
    }
}

CAN_TxHeaderTypeDef Ctx;
/**
*  @brief Send_GM6020_Motor_Message
*
*  @author CGH
*/
void Send_GM6020_Motor_Message(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box; 
    Ctx.StdId = CAN_CHASSIS_ALL_ID; //标识符
    Ctx.IDE = CAN_ID_STD; //标准发送格式
    Ctx.RTR = CAN_RTR_DATA; //数据帧
		Ctx.DLC = 0x08; //一次发送的数据的字节数
    chassis_can_send_data[0] = motor1 >> 8; 
    chassis_can_send_data[1] = motor1; 
    chassis_can_send_data[2] = motor2 >> 8; 
    chassis_can_send_data[3] = motor2; 
    chassis_can_send_data[4] = motor3 >> 8; 
    chassis_can_send_data[5] = motor3; 
    chassis_can_send_data[6] = motor4 >> 8; 
    chassis_can_send_data[7] = motor4; 
 
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &Ctx, chassis_can_send_data, &send_mail_box); 

    HAL_CAN_AddTxMessage(&CHASSIS_CAN2, &Ctx, chassis_can_send_data, &send_mail_box); 

}

