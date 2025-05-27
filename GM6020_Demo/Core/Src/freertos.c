/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "dri_uart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_MAGIC 0x00FF1234    //消息头用于分别数据，此处表示左开关
#define MSG_MAGIC2 0x00FF1235//用于分别数据，表示是yaw轴
#define MSG_MAGIC3 0x00FF1236//pitch轴
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//extern QueueHandle_t Usb_quene;
extern Moto_GM6020_t GM6020;
extern Moto_GM6020_t GM6020_pitch;
extern PID PosePID_yaw;
extern PID PosePID_pitch;
extern uint8_t CAN_Input;
extern PID VelPID_yaw;
extern PID VelPID_pitch;

extern CAN_RxHeaderTypeDef rx_header;//can总线接受的接收区头
extern uint8_t rx_data[8];
BaseType_t xStatus2;
//uint16_t target=100;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GM6020_TaskHandle;
osThreadId GM6020_Task_innHandle;
osThreadId CAN_input_taskHandle;
osMessageQId Usb_queneHandle;
osMessageQId RCqueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);
void StartTask04(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Usb_quene */
  osMessageQDef(Usb_quene, 32, uint32_t);
  Usb_queneHandle = osMessageCreate(osMessageQ(Usb_quene), NULL);

  /* definition and creation of RCqueue */
  osMessageQDef(RCqueue, 16, uint32_t);
  RCqueueHandle = osMessageCreate(osMessageQ(RCqueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GM6020_Task */
  osThreadDef(GM6020_Task, StartTask02, osPriorityAboveNormal, 0, 512);
  GM6020_TaskHandle = osThreadCreate(osThread(GM6020_Task), NULL);

  /* definition and creation of GM6020_Task_inn */
  osThreadDef(GM6020_Task_inn, StartTask03, osPriorityAboveNormal, 0, 512);
  GM6020_Task_innHandle = osThreadCreate(osThread(GM6020_Task_inn), NULL);

  /* definition and creation of CAN_input_task */
  osThreadDef(CAN_input_task, StartTask04, osPriorityRealtime, 0, 128);
  CAN_input_taskHandle = osThreadCreate(osThread(CAN_input_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	uint8_t minipc_tx_buff[18];
  for(;;)
  {
	//CDC_Transmit_FS(minipc_tx_buff, sizeof(minipc_tx_buff));
	VOFA_Tx();
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the GM6020_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint32_t received_target_angle;
	float received_traget_angle_f;
	uint32_t temp_message;
	 BaseType_t xStatus1;
	BaseType_t xStatus2;
	uint8_t emergence_stop=0;
	 const TickType_t xTicksToWait = 0;//静态变量
	 init_sine_generator(0.0f);//初始化正弦波发生器
	
  /* Infinite loop */
  for(;;)
  {
		//float target_position = generate_sine_target();
		xStatus1 = xQueueReceive(RCqueueHandle, &temp_message, xTicksToWait);
		  // 检查是否成功接收到数据
    if (xStatus1 == pdPASS)
    {
		if(temp_message==MSG_MAGIC){
				xQueueReceive(RCqueueHandle, &temp_message, xTicksToWait);

				if(temp_message==1){
				Send_GM6020_Motor_Message(0x00, 0x00, 0x00, 0x00);
					emergence_stop=1;
				}else{
					emergence_stop=0;
				}
				
			}else if(temp_message==MSG_MAGIC2){
				xQueueReceive(RCqueueHandle, &received_target_angle, xTicksToWait);
				received_traget_angle_f=(float)received_target_angle;
				GM6020.Set_Angle+=(received_traget_angle_f-1024)*0.1;
				
				if(GM6020.Set_Angle>=8192){
					GM6020.Set_Angle=0;
				}
				if(GM6020.Set_Angle<0){
					GM6020.Set_Angle+=8191;
				}
			}
			}
				//////////pitch轴电机控制/////////////
					//float target_position = generate_sine_target();
		xStatus1 = xQueueReceive(RCqueueHandle, &temp_message, xTicksToWait);
		  // 检查是否成功接收到数据
    if (xStatus1 == pdPASS)
    {
		if(temp_message==MSG_MAGIC){
				xQueueReceive(RCqueueHandle, &temp_message, xTicksToWait);

				if(temp_message==1){
				Send_GM6020_Motor_Message(0x00, 0x00, 0x00, 0x00);
					emergence_stop=1;
				}else{
					emergence_stop=0;
				}
				
			}else if(temp_message==MSG_MAGIC3){
					xQueueReceive(RCqueueHandle, &received_target_angle, xTicksToWait);
				received_traget_angle_f=(float)received_target_angle;
				GM6020_pitch.Set_Angle+=(received_traget_angle_f-1024)*0.1;
				
				if(GM6020_pitch.Set_Angle>=2330){
					GM6020_pitch.Set_Angle=2330;
				}
				if(GM6020_pitch.Set_Angle<1090){
					GM6020_pitch.Set_Angle=1090;
				}
			}
			
			
	  }
		//GM6020.Set_Angle = generate_sine_target();
		//float temp_result1=position_PID(target_position,GM6020.rotor_angle);
		float temp_result1=position_PID(GM6020.Set_Angle,GM6020.rotor_angle,&PosePID_yaw);
		GM6020.test=temp_result1;
		float temp_result2=position_PID(GM6020_pitch.Set_Angle,GM6020_pitch.rotor_angle,&PosePID_pitch);
		if (emergence_stop != 1) {
    // 发送消息类型标识符
    uint32_t msg_type = MSG_MAGIC2;
    xStatus2=xQueueSend(Usb_queneHandle, &msg_type, xTicksToWait);
    
    // 发送yaw轴数据
    uint32_t raw_data = *(uint32_t*)&temp_result1;
    xQueueSend(Usb_queneHandle, &raw_data, xTicksToWait);
    
    // 发送另一轴数据
    msg_type = MSG_MAGIC3;
    xQueueSend(Usb_queneHandle, &msg_type, xTicksToWait);
    raw_data = *(uint32_t*)&temp_result2;
    xQueueSend(Usb_queneHandle, &raw_data, xTicksToWait);
    }
		if(xStatus2!=pdPASS){
     UBaseType_t messages_waiting = uxQueueMessagesWaiting(Usb_queneHandle);
			printf("队列已满！当前消息数：%lu，%d）\r\n", messages_waiting, 32); // 32 是你的队列容量(并不是32强鸽鸽）
		}
    vTaskDelay(pdMS_TO_TICKS(1)); 
		
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the GM6020_Task_inn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint32_t received_target_velocity;
	uint32_t received_target_velocity2;
	float received_target_velocity_f;
  float received_target_velocity2_f;	// 用于存储从队列接收到的目标速度2
  BaseType_t xStatus;             // 用于检查 xQueueReceive 的返回值
	uint32_t temp_message;
  const TickType_t xTicksToWait = 0; // 设置等待时间
  /* Infinite loop */
  for(;;)
  {
		xStatus = xQueueReceive(Usb_queneHandle, &temp_message, xTicksToWait);
		if(temp_message==MSG_MAGIC2){
				xStatus =xQueueReceive(Usb_queneHandle, &received_target_velocity, xTicksToWait);
				received_target_velocity_f=*(float*)&received_target_velocity;
				xStatus =xQueueReceive(Usb_queneHandle, &temp_message, xTicksToWait);
			if(temp_message==MSG_MAGIC3){
				xQueueReceive(Usb_queneHandle, &received_target_velocity2, xTicksToWait);
		    received_target_velocity2_f=*(float*)&received_target_velocity2;
				
			}
		}
		
		  // 检查是否成功接收到数据
    if (xStatus == pdPASS)
    {
      // 成功接收到数据，现在 received_target_velocity 包含了 Task02 发送的值
      // 使用接收到的值作为速度PID的目标值
			
			received_target_velocity_f=(received_target_velocity_f>=340)?340:received_target_velocity_f;
			received_target_velocity_f=(received_target_velocity_f<=-340)?-340:received_target_velocity_f;
			
			GM6020.Set_Speed=received_target_velocity_f;
			received_target_velocity2_f=(received_target_velocity2_f>=340)?340:received_target_velocity2_f;
			received_target_velocity2_f=(received_target_velocity2_f<=-340)?-340:received_target_velocity2_f;
			//float temp_result = velocity_PID(received_target_velocity_f, GM6020.rotor_speed,&VelPID_yaw);
      float temp_result2 = velocity_PID(received_target_velocity2_f, GM6020_pitch.rotor_speed,&VelPID_pitch);
      float temp_result = velocity_PID(GM6020.Set_Speed, GM6020.rotor_speed,&VelPID_yaw);
			//GM6020.test=temp_result;
      
      // temp_result2 （目标电压）需要转换类型。
      int16_t motor_command = (int16_t)temp_result; 
      motor_command = (motor_command > 25000) ? 25000 : motor_command; // 限制上限
      motor_command = (motor_command < -25000) ? -25000 : motor_command; // 限制下限
			int16_t motor_command2 = (int16_t)temp_result2; 
      motor_command2 = (motor_command2 > 25000) ? 25000 : motor_command2; // 限制上限
      motor_command2 = (motor_command2 < -25000) ? -25000 : motor_command2; // 限制下限
      GM6020.test=motor_command;//第一个电机控制的是yaw轴喔
			Send_GM6020_Motor_Message(motor_command2, 0x00,motor_command, 0x00); 
    }
    else
    {
      // 未能从队列接收到数据 
      // 错误处理逻辑，发送一个安全的电机指令（0）
      Send_GM6020_Motor_Message(0x00, 0x00, 0x00, 0x00);
      // printf("Failed to receive from queue\n"); // 调试信息
    }
		osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the CAN_input_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  { 
		
   
    if (CAN_Input == 1) {
      /* 信号到达后的处理流程 */
      Get_GM6020_Motor_Message(rx_header.StdId,rx_data);
      CAN_Input=0;
      /* 处理完成后自动回到循环开头，再次进入阻塞 */
    }
		
	
		
    osDelay(1);
  }
  /* USER CODE END StartTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
