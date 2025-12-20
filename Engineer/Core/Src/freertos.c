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
#include "chassis.h"
#include "arm.h"
#include "shoot.h"
#include "ins_task.h"
#include "DJI_motor.h"
#include "usb.h"
#include "Test.h"
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
osThreadId ImuTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId UsbTaskHandle;
osThreadId ArmTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ImuTask_f(void const * argument);
void ChassisTask_f(void const * argument);
void Usb_f(void const * argument);
void ArmTask_f(void const * argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ImuTask */
  osThreadDef(ImuTask, ImuTask_f, osPriorityAboveNormal, 0, 512);
  ImuTaskHandle = osThreadCreate(osThread(ImuTask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisTask_f, osPriorityAboveNormal, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* definition and creation of UsbTask */
  osThreadDef(UsbTask, Usb_f, osPriorityAboveNormal, 0, 512);
  UsbTaskHandle = osThreadCreate(osThread(UsbTask), NULL);

  /* definition and creation of ArmTask */
  osThreadDef(ArmTask, ArmTask_f, osPriorityAboveNormal, 0, 512);
  ArmTaskHandle = osThreadCreate(osThread(ArmTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ImuTask_f */
/**
  * @brief  Function implementing the ImuTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ImuTask_f */
void ImuTask_f(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN ImuTask_f */
  /* Infinite loop */
  for(;;)
  {
		INS_Task();
    osDelay(1);
  }
  /* USER CODE END ImuTask_f */
}

/* USER CODE BEGIN Header_ChassisTask_f */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisTask_f */
void ChassisTask_f(void const * argument)
{
  /* USER CODE BEGIN ChassisTask_f */
  /* Infinite loop */
  for(;;){
//  { HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
		all_cmd_Task();
    osDelay(1);
  }
  /* USER CODE END ChassisTask_f */
}

/* USER CODE BEGIN Header_Usb_f */
/**
* @brief Function implementing the UsbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Usb_f */
void Usb_f(void const * argument)
{
  /* USER CODE BEGIN Usb_f */
  /* Infinite loop */
  for(;;)
  {
		USB_ProcessTask();
    osDelay(1);
  }
  /* USER CODE END Usb_f */
}

/* USER CODE BEGIN Header_ArmTask_f */
/**
* @brief Function implementing the ArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ArmTask_f */
void ArmTask_f(void const * argument)
{
  /* USER CODE BEGIN ArmTask_f */
  /* Infinite loop */
  for(;;)
  {
//     ArmTask();
//		 
  }
  /* USER CODE END ArmTask_f */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
