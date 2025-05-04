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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
//extern QueueHandle_t Usb_quene;
extern Moto_GM6020_t GM6020;
//uint16_t target=100;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId GM6020_TaskHandle;
osThreadId GM6020_Task_innHandle;
osMessageQId Usb_queneHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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
  osMessageQDef(Usb_quene, 16, uint32_t);
  Usb_queneHandle = osMessageCreate(osMessageQ(Usb_quene), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of GM6020_Task */
  osThreadDef(GM6020_Task, StartTask02, osPriorityBelowNormal, 0, 512);
  GM6020_TaskHandle = osThreadCreate(osThread(GM6020_Task), NULL);

  /* definition and creation of GM6020_Task_inn */
  osThreadDef(GM6020_Task_inn, StartTask03, osPriorityNormal, 0, 512);
  GM6020_Task_innHandle = osThreadCreate(osThread(GM6020_Task_inn), NULL);

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
	 BaseType_t xStatus;
	 const TickType_t xTicksToWait = 0;//静态变量
	 init_sine_generator(0.0f);//初始化正弦波发生器
	
  /* Infinite loop */
  for(;;)
  {
		//float target_position = generate_sine_target();
		GM6020.Set_Angle = generate_sine_target();
		//float temp_result1=position_PID(target_position,GM6020.rotor_angle);
		float temp_result1=position_PID(GM6020.Set_Angle,GM6020.rotor_angle);
		

    xStatus = xQueueSend(Usb_queneHandle, &temp_result1, xTicksToWait);
    if (xStatus != pdPASS)
    {
        // 队列已满或发送失败
        printf("Warning: Failed to send to Usb_queue\r\n");
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
	float received_target_velocity; // 用于存储从队列接收到的目标速度2
  BaseType_t xStatus;             // 用于检查 xQueueReceive 的返回值
	
  const TickType_t xTicksToWait = portMAX_DELAY; // 设置等待时间
  /* Infinite loop */
  for(;;)
  {
		xStatus = xQueueReceive(Usb_queneHandle, &received_target_velocity, xTicksToWait);
		  // 检查是否成功接收到数据
    if (xStatus == pdPASS)
    {
      // 成功接收到数据，现在 received_target_velocity 包含了 Task02 发送的值
      // 使用接收到的值作为速度PID的目标值
			received_target_velocity=(received_target_velocity>=340)?340:received_target_velocity;
			received_target_velocity=(received_target_velocity<=-340)?-340:received_target_velocity;
			
      float temp_result2 = velocity_PID(received_target_velocity, GM6020.rotor_speed);
        //float temp_result2 = velocity_PID(GM6020.Set_Speed, GM6020.rotor_speed);
      
      // temp_result2 （目标电压）需要转换类型。
      int16_t motor_command = (int16_t)temp_result2; 
      motor_command = (motor_command > 25000) ? 25000 : motor_command; // 限制上限
      motor_command = (motor_command < -25000) ? -25000 : motor_command; // 限制下限
      GM6020.test=motor_command;
			Send_GM6020_Motor_Message(motor_command,0x00, 0x00, 0x00); 
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
