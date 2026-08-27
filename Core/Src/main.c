/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "spi.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RS485.h"
#include "MS5314.h"
#include "queue.h"
#include "timers.h"
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

/* USER CODE BEGIN PV */
uint8_t rx_buffer[RX_BUFFER_SIZE];
extern xQueueHandle rs485_queue;
extern TimerHandle_t xHallSensorTimer[3];
//config_info ConfigInfo = {.state = 0, .blade_power_levels = {{0}}};
//config_info ConfigInfo1 = {.blade_power_levels = {{1, 1}, {2, 2}, {3, 3}}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  SCB->VTOR = APP_FLASH_STARTADDR;
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  bsp_init();
  //HAL_Delay(6);
  //__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);    //使能UART2空闲中断
  //__enable_irq();
  //HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);

  /* PWR_PVDTypeDef sConfigPVD = {.Mode = PWR_PVD_MODE_IT_RISING_FALLING, .PVDLevel = PWR_PVDLEVEL_7};
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_ConfigPVD(&sConfigPVD);
  HAL_PWR_EnablePVD();
  HAL_NVIC_SetPriority(PVD_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(PVD_IRQn); */

  //HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_RESET);
  //ConfigInfo_Save(&ConfigInfo1);
  //ConfigInfo_ResetToDefault();
  //ConfigInfo_Load(&ConfigInfo);
  /* if (ConfigInfo.state)
    HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET); */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    // Handle UART2 idle event
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    RS485_QueueMsg msg;
    if (RS485_ProcessFrame(rx_buffer, Size, &msg) == true)
    {
        // Process the valid frame
        xQueueSendFromISR(rs485_queue, &msg, &xHigherPriorityTaskWoken);
    }
    else
    {

    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, rx_buffer, RX_BUFFER_SIZE);    //重新启动下一次接 ?
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t hall_channel = 0;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  switch (GPIO_Pin)
  {
  case HALL_IN1_Pin:
  {
    hall_channel = 0;
    Laser_Enable(BLADE_NUM1);
    Laser_Set_Brightness(BLADE_NUM1, LASER_BRIGHTNESS_LEVEL1);
    break;
  }
  case HALL_IN2_Pin:
  {
    hall_channel = 1;
    Laser_Enable(BLADE_NUM2);
    Laser_Set_Brightness(BLADE_NUM2, LASER_BRIGHTNESS_LEVEL1);
    break;
  }
  case HALL_IN3_Pin:
  {
    hall_channel = 2;
    Laser_Enable(BLADE_NUM3);
    Laser_Set_Brightness(BLADE_NUM3, LASER_BRIGHTNESS_LEVEL1);
    break;
  }
  default:
    break;
  }
  /* if (xTimerIsTimerActiveFromISR(xHallSensorTimer[hall_channel]) == pdFALSE)
  {
    xTimerStopFromISR(xHallSensorTimer[hall_channel], &xHigherPriorityTaskWoken);
  } */
  xTimerStartFromISR(xHallSensorTimer[hall_channel], &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_PWR_PVDCallback(void)
{
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_PVDO) != RESET)
  {
    config_info ConfigInfo1 = {.blade_power_levels = {{10, 10}, {2, 2}, {3, 3}}};
    /* __HAL_PWR_CLEAR_FLAG(PWR_FLAG_PVDO);
    __disable_irq(); */
    if (ConfigInfo_Save(&ConfigInfo1))
      HAL_GPIO_WritePin(LED_STATE_GPIO_Port, LED_STATE_Pin, GPIO_PIN_SET);
    /* __enable_irq(); */
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
