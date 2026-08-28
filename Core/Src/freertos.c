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
#include "timers.h"
#include "semphr.h"
#include "iwdg.h"
#include "RS485.h"
#include "bsp.h"
#include "MS5314.h"
#include "adc_dma.h"
#include "flash_drv.h"
#include "ntc.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
xQueueHandle rs485_queue;
TaskHandle_t xRS485CommandTaskHandle = NULL;

TimerHandle_t xHallSensorTimer[3];

SemaphoreHandle_t xTemperatureReadSemaphore = NULL;
TimerHandle_t xTemperatureReadTimer = NULL;

TimerHandle_t xWatchdogTimer = NULL;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
sys_info system_info;
extern uint8_t blade_numbers[BLADE_NUMS];
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

void HallSensorTimerCallback(TimerHandle_t xTimer);

void TemperatureReadTimerCallback(TimerHandle_t xTimer);
void TemperatureReadTask(void *argument);

void WatchdogTimerCallback(TimerHandle_t xTimer);
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
  xTemperatureReadSemaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  xHallSensorTimer[0] = xTimerCreate("HallSensorTimer1", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, HallSensorTimerCallback);
  xHallSensorTimer[1] = xTimerCreate("HallSensorTimer2", pdMS_TO_TICKS(1000), pdFALSE, (void *)1, HallSensorTimerCallback);
  xHallSensorTimer[2] = xTimerCreate("HallSensorTimer3", pdMS_TO_TICKS(1000), pdFALSE, (void *)2, HallSensorTimerCallback);

  xTemperatureReadTimer = xTimerCreate("TemperatureReadTimer", pdMS_TO_TICKS(3000), pdTRUE, NULL, TemperatureReadTimerCallback);
  if (xTimerStart(xTemperatureReadTimer, 0) != pdPASS)    //start timer
  {
    // Handle error in starting the timer
    while (1)
    {}
  }

  xWatchdogTimer = xTimerCreate("WatchdogTimer", pdMS_TO_TICKS(2000), pdTRUE, NULL, WatchdogTimerCallback);
  if (xTimerStart(xWatchdogTimer, 0) != pdPASS)
  {
    while (1)
    {
      
    }
    
  }
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

  xReturned = xTaskCreate(TemperatureReadTask, "TemperatureReadTask", 128, NULL, osPriorityNormal2, NULL);
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
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(500));
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
        /* if (Laser_Set_Brightness(BladeNums, LASER_BRIGHTNESS_LEVEL0))
        { */
          system_info.all_laser_status &= (~BladeNums);
          for (uint8_t blade_idx = 0; blade_idx < BLADE_NUMS; blade_idx++)
          {
              if (BladeNums & blade_numbers[blade_idx])
              {
                  system_info.each_laser_status[blade_idx] = 0;
              }
          }
          RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_OFF, 1, &system_info.all_laser_status);
        /* } */
        /* else
        {
          // Handle error in setting brightness
        } */

        break;
      }
      case RS485_FRAME_CMD_ON:
      {
        BladeNums = msg.data[0];
        //bool LaserSetResult = Laser_Set_Brightness(BladeNums, LASER_BRIGHTNESS_LEVEL1);
        Laser_Enable(BladeNums);
        //if (LaserSetResult)
        //{
          system_info.all_laser_status |= BladeNums;
          for (uint8_t blade_idx = 0; blade_idx < BLADE_NUMS; blade_idx++)
          {
              if (BladeNums & blade_numbers[blade_idx])
              {
                  system_info.each_laser_status[blade_idx] = 0x01;
              }
          }
          RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_ON, 1, &system_info.all_laser_status);
        //}
        /* else
        {
          // Handle error in setting brightness
        } */

        break;
      }
      case RS485_FRAME_CMD_SET_CONTROL_MODE:
      {
        system_info.control_mode = msg.data[0];
        
        if (system_info.control_mode == CONTROL_MODE_SERIAL)
        {
          HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
          HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
          Laser_Enable(BLADE_NUM_ALL);
        }
        else if (system_info.control_mode == CONTROL_MODE_MANUAL)
        {
          HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
          HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
          Laser_Disable(BLADE_NUM_ALL);
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
        uint8_t blade_num = msg.data[0], power_levels[6];
        uint16_t powerlevel = (msg.data[1] << 8) | msg.data[2]; // Combine two bytes to form a 16-bit power level
        Laser_Set_PowerLevel(blade_num, powerlevel);

        for (uint8_t blade_idx = 0; blade_idx < BLADE_NUMS; blade_idx++)
        {
            if (blade_num & blade_numbers[blade_idx])
            {
                system_info.blade_power_level[blade_idx][0] = msg.data[1];
                system_info.blade_power_level[blade_idx][1] = msg.data[2];
            }
        }
        for (uint8_t i = 0; i < BLADE_NUMS; i++)
        {
            memcpy(power_levels + (i * 2), system_info.blade_power_level[i], 2);
        }
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_SET_POWER_LEVEL, 6, power_levels);

        break;
      }
      case RS485_FRAME_CMD_GET_POWER_LEVEL:
      {
        uint8_t power_levels[6] = {system_info.blade_power_level[0][0], system_info.blade_power_level[0][1],
                                  system_info.blade_power_level[1][0], system_info.blade_power_level[1][1],
                                  system_info.blade_power_level[2][0], system_info.blade_power_level[2][1]};
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_GET_POWER_LEVEL, 6, power_levels);

        break;
      }
      case RS485_FRAME_CMD_GET_DEVICE_INFO:
      {
        uint8_t device_info[23];
        memcpy(device_info, system_info.blade_power_level[0], 2);
        memcpy(device_info + 2, system_info.blade_power_level[1], 2);
        memcpy(device_info + 4, system_info.blade_power_level[2], 2);
        memcpy(device_info + 6, system_info.serial_number, 11);
        memcpy(device_info + 17, system_info.firmware_version, 6);
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_GET_DEVICE_INFO, 23, device_info);

        break;
      }
      case RS485_FRAME_CMD_GET_SYS_STATE:
      {
        uint8_t sys_state[21];

        for (uint8_t i = 0; i < BLADE_NUMS; i++)
        {
            memcpy(sys_state + (i * 4), &system_info.laser_temperature[i], 4);
            //memcpy(sys_state + 12 + (i * 2), system_info.blade_power_level[i], 2);
            sys_state[12 + i * 2] = system_info.blade_power_level[i][1];
            sys_state[12 + i * 2 + 1] = system_info.blade_power_level[i][0];
            memcpy(sys_state + 18 + i, &system_info.each_laser_status[i], 1);
        }
        RS485_RespondFrame(RS485_NUM1 | RS485_NUM2, RS485_FRAME_CMD_GET_SYS_STATE, 21, sys_state);

        break;
      }
      case RS485_FRAME_CMD_FIRMWARE_UPGRADE:
      {
        uint32_t upgrade_flag_magic = FIRMWIRE_UPGRADE_FLAGE_MAGIC;
        
        if (Flash_WriteBuffer(FIRMWIRE_UPGRADE_FLAG_ADDR, &upgrade_flag_magic, 1, true) == false)
          return;
        NVIC_SystemReset();

        break;
      }
      default:
        break;
      }
    } 
  }
}

void TemperatureReadTask(void *argument)
{
  while (1)
  {
    if (xSemaphoreTake(xTemperatureReadSemaphore, portMAX_DELAY) == pdTRUE)
    {
      ADC_DMA_Start();

      uint32_t timeout = pdMS_TO_TICKS(100);
      TickType_t start = xTaskGetTickCount();

      while (!ADC_DMA_IsComplete())
      {
        if ((xTaskGetTickCount() - start) > timeout)
        {
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      if (ADC_DMA_IsComplete())
      {
        float temperature[ADC_DMA_BUFFER_SIZE];
        NTC_GetTemperature(temperature);
        system_info.laser_temperature[0] = temperature[0];
        system_info.laser_temperature[1] = temperature[1];
        system_info.laser_temperature[2] = temperature[2];
      }
    }
  }
}

void HallSensorTimerCallback(TimerHandle_t xTimer)
{
    uint32_t timer_id = (uint32_t)pvTimerGetTimerID(xTimer);
    if (timer_id < 3)
    {
        Laser_Disable(blade_numbers[timer_id]);
        system_info.all_laser_status &= (~blade_numbers[timer_id]);
        system_info.each_laser_status[timer_id] = 0;
    }
}

void TemperatureReadTimerCallback(TimerHandle_t xTimer)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(xTemperatureReadSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void WatchdogTimerCallback(TimerHandle_t xTimer)
{
  HAL_IWDG_Refresh(&hiwdg);
}

/* USER CODE END Application */

