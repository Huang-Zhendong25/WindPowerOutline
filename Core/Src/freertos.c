/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "RS485.h"
#include "bsp.h"
#include "MS5314.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
xQueueHandle rs485_queue;
TaskHandle_t xRS485CommandTaskHandle = NULL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
sys_info system_info;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void RS485CommandTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  rs485_queue = xQueueCreate(10, sizeof(RS485_QueueMsg));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  BaseType_t xReturned = xTaskCreate(RS485CommandTask, "RS485CommandTask", 128, NULL, osPriorityHigh, &xRS485CommandTaskHandle);
  if (xReturned != pdPASS)
  {
    /* Task creation failed */
    while (1)
    {}
  }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1000 milliseconds (1 second)
    HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1000 milliseconds (1 second)
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void RS485CommandTask(void *argument)
{
  RS485_QueueMsg msg;

  while (1)
  {
    if (xQueueReceive(rs485_queue, &msg, portMAX_DELAY) == pdTRUE)
    {
      uint8_t BladeNums;
      // Process the received message
      switch (msg.cmd)
      {
      case RS485_FRAME_CMD_OFF:
      {
        BladeNums = msg.data[0];
        Laser_Disable(BladeNums);
        if (Laser_Set_Brightness(BladeNums, LASER_BRIGHTNESS_LEVEL0))
        {
          system_info.laser_status &= (~BladeNums);
          RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_OFF, 1, &system_info.laser_status);
        }
        else
        {
          // Handle error in setting brightness
        }

        break;
      }
      case RS485_FRAME_CMD_ON:
      {
        BladeNums = msg.data[0];
        bool LaserSetResult = Laser_Set_Brightness(BladeNums, LASER_BRIGHTNESS_LEVEL1);
        Laser_Enable(BladeNums);
        if (LaserSetResult)
        {
          system_info.laser_status |= BladeNums;
          RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_ON, 1, &system_info.laser_status);
        }
        else
        {
          // Handle error in setting brightness
        }

        break;
      }
      case RS485_FRAME_CMD_SET_CONTROL_MODE:
      {
        system_info.control_mode = msg.data[0];
        
        if (system_info.control_mode == CONTROL_MODE_SERIAL)
        {
          HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
          HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
        }
        else if (system_info.control_mode == CONTROL_MODE_MANUAL)
        {
          HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
          HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
        }
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_SET_CONTROL_MODE, 1, &system_info.control_mode);

        break;
      }
      case RS485_FRAME_CMD_GET_CONTROL_MODE:
      {
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_GET_CONTROL_MODE, 1, &system_info.control_mode);
        break;
      }
      case RS485_FRAME_CMD_SET_POWER_LEVEL:
      {
        
        break;
      }
      case RS485_FRAME_CMD_GET_POWER_LEVEL:
      {
        uint8_t power_levels[6] = {system_info.blade1_power_level[0], system_info.blade1_power_level[1],
                                  system_info.blade2_power_level[0], system_info.blade2_power_level[1],
                                  system_info.blade3_power_level[0], system_info.blade3_power_level[1]};
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_GET_POWER_LEVEL, 6, power_levels);
        break;
      }
      case RS485_FRAME_CMD_GET_DEVICE_INFO:
      {
        break;
      }
      default:
        break;
      }
    } 
  }
  
}

/* USER CODE END Application */

