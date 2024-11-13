/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
osThreadId defaultTaskHandle;
osThreadId Imu_TaskHandle;
uint32_t Imu_TaskBuffer[ 2048 ];
osStaticThreadDef_t Imu_TaskControlBlock;
osThreadId Boat_TaskHandle;
uint32_t Boat_TaskBuffer[ 1024 ];
osStaticThreadDef_t Boat_TaskControlBlock;
osThreadId Trans_TaskHandle;
uint32_t Trans_TaskBuffer[ 1024 ];
osStaticThreadDef_t Trans_TaskControlBlock;
osThreadId Motor_TaskHandle;
uint32_t Motor_TaskBuffer[ 2048 ];
osStaticThreadDef_t Motor_TaskControlBlock;
osThreadId Chassis_TaskHandle;
uint32_t Chassis_TaskBuffer[ 1024 ];
osStaticThreadDef_t Chassis_TaskControlBlock;
osThreadId Gimbal_TaskHandle;
uint32_t Gimbal_TaskBuffer[ 1024 ];
osStaticThreadDef_t Gimbal_TaskControlBlock;
osThreadId Shoot_TaskHandle;
uint32_t Shoot_TaskBuffer[ 1024 ];
osStaticThreadDef_t Shoot_TaskControlBlock;
osThreadId Referee_TaskHandle;
uint32_t Referee_TaskBuffer[ 2048 ];
osStaticThreadDef_t Referee_TaskControlBlock;
osThreadId Cmd_TaskHandle;
uint32_t Cmd_TaskBuffer[ 1024 ];
osStaticThreadDef_t Cmd_TaskControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Imu_Task_Entry(void const * argument);
void Boat_Task_Entry(void const * argument);
void Trans_Task_Entry(void const * argument);
void Motor_Task_Entry(void const * argument);
void Chassis_Task_Entry(void const * argument);
void Gimbal_Task_Entry(void const * argument);
void Shoot_Task_Entry(void const * argument);
void Referee_Task_Entry(void const * argument);
void Cmd_Task_Entry(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Imu_Task */
  osThreadStaticDef(Imu_Task, Imu_Task_Entry, osPriorityAboveNormal, 0, 2048, Imu_TaskBuffer, &Imu_TaskControlBlock);
  Imu_TaskHandle = osThreadCreate(osThread(Imu_Task), NULL);

  /* definition and creation of Boat_Task */
  osThreadStaticDef(Boat_Task, Boat_Task_Entry, osPriorityNormal, 0, 1024, Boat_TaskBuffer, &Boat_TaskControlBlock);
  Boat_TaskHandle = osThreadCreate(osThread(Boat_Task), NULL);

  /* definition and creation of Trans_Task */
  osThreadStaticDef(Trans_Task, Trans_Task_Entry, osPriorityNormal, 0, 1024, Trans_TaskBuffer, &Trans_TaskControlBlock);
  Trans_TaskHandle = osThreadCreate(osThread(Trans_Task), NULL);

  /* definition and creation of Motor_Task */
  osThreadStaticDef(Motor_Task, Motor_Task_Entry, osPriorityNormal, 0, 2048, Motor_TaskBuffer, &Motor_TaskControlBlock);
  Motor_TaskHandle = osThreadCreate(osThread(Motor_Task), NULL);

  /* definition and creation of Chassis_Task */
  osThreadStaticDef(Chassis_Task, Chassis_Task_Entry, osPriorityNormal, 0, 1024, Chassis_TaskBuffer, &Chassis_TaskControlBlock);
  Chassis_TaskHandle = osThreadCreate(osThread(Chassis_Task), NULL);

  /* definition and creation of Gimbal_Task */
  osThreadStaticDef(Gimbal_Task, Gimbal_Task_Entry, osPriorityNormal, 0, 1024, Gimbal_TaskBuffer, &Gimbal_TaskControlBlock);
  Gimbal_TaskHandle = osThreadCreate(osThread(Gimbal_Task), NULL);

  /* definition and creation of Shoot_Task */
  osThreadStaticDef(Shoot_Task, Shoot_Task_Entry, osPriorityNormal, 0, 1024, Shoot_TaskBuffer, &Shoot_TaskControlBlock);
  Shoot_TaskHandle = osThreadCreate(osThread(Shoot_Task), NULL);

  /* definition and creation of Referee_Task */
  osThreadStaticDef(Referee_Task, Referee_Task_Entry, osPriorityNormal, 0, 2048, Referee_TaskBuffer, &Referee_TaskControlBlock);
  Referee_TaskHandle = osThreadCreate(osThread(Referee_Task), NULL);

  /* definition and creation of Cmd_Task */
  osThreadStaticDef(Cmd_Task, Cmd_Task_Entry, osPriorityNormal, 0, 1024, Cmd_TaskBuffer, &Cmd_TaskControlBlock);
  Cmd_TaskHandle = osThreadCreate(osThread(Cmd_Task), NULL);

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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Imu_Task_Entry */
/**
* @brief Function implementing the Imu_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Imu_Task_Entry */
__weak void Imu_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Imu_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Imu_Task_Entry */
}

/* USER CODE BEGIN Header_Boat_Task_Entry */
/**
* @brief Function implementing the Boat_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Boat_Task_Entry */
__weak void Boat_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Boat_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Boat_Task_Entry */
}

/* USER CODE BEGIN Header_Trans_Task_Entry */
/**
* @brief Function implementing the Trans_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Trans_Task_Entry */
__weak void Trans_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Trans_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Trans_Task_Entry */
}

/* USER CODE BEGIN Header_Motor_Task_Entry */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Task_Entry */
__weak void Motor_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Motor_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_Task_Entry */
}

/* USER CODE BEGIN Header_Chassis_Task_Entry */
/**
* @brief Function implementing the Chassis_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task_Entry */
__weak void Chassis_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task_Entry */
}

/* USER CODE BEGIN Header_Gimbal_Task_Entry */
/**
* @brief Function implementing the Gimbal_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task_Entry */
__weak void Gimbal_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task_Entry */
}

/* USER CODE BEGIN Header_Shoot_Task_Entry */
/**
* @brief Function implementing the Shoot_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task_Entry */
__weak void Shoot_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task_Entry */
}

/* USER CODE BEGIN Header_Referee_Task_Entry */
/**
* @brief Function implementing the Referee_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task_Entry */
__weak void Referee_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Referee_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task_Entry */
}

/* USER CODE BEGIN Header_Cmd_Task_Entry */
/**
* @brief Function implementing the Cmd_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Cmd_Task_Entry */
__weak void Cmd_Task_Entry(void const * argument)
{
  /* USER CODE BEGIN Cmd_Task_Entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Cmd_Task_Entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
