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
#include "Usart1.h"
#include "usart.h"
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

/* USER CODE END Variables */
osThreadId LED_GHandle;
osThreadId LED_RHandle;
osThreadId UsartTaskHandle;
osMessageQId RCqueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void LED_GTask(void const * argument);
void LED_RTask(void const * argument);
void usart_Task(void const * argument);

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
  /* definition and creation of RCqueue */
  osMessageQDef(RCqueue, 16, uint16_t);
  RCqueueHandle = osMessageCreate(osMessageQ(RCqueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LED_G */
  osThreadDef(LED_G, LED_GTask, osPriorityLow, 0, 256);
  LED_GHandle = osThreadCreate(osThread(LED_G), NULL);

  /* definition and creation of LED_R */
  osThreadDef(LED_R, LED_RTask, osPriorityLow, 0, 128);
  LED_RHandle = osThreadCreate(osThread(LED_R), NULL);

  /* definition and creation of UsartTask */
  osThreadDef(UsartTask, usart_Task, osPriorityNormal, 0, 128);
  UsartTaskHandle = osThreadCreate(osThread(UsartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LED_GTask */
/**
  * @brief  Function implementing the LED_G thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LED_GTask */
__weak void LED_GTask(void const * argument)
{
  /* USER CODE BEGIN LED_GTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_GTask */
}

/* USER CODE BEGIN Header_LED_RTask */
/**
* @brief Function implementing the LED_R thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_RTask */
__weak void LED_RTask(void const * argument)
{
  /* USER CODE BEGIN LED_RTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_RTask */
}

/* USER CODE BEGIN Header_usart_Task */
/**
* @brief Function implementing the UsartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usart_Task */
void usart_Task(void const * argument)
{
  /* USER CODE BEGIN usart_Task */
  /* Infinite loop */
  for(;;)
  {
		usart_Send();
		// FreeRTOS延时（使用时间转换宏）
		osDelay(500);
		//vTaskDelay(pdMS_TO_TICKS(1000));  
    
  }
  /* USER CODE END usart_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
