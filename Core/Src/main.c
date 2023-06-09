/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timers.h"
#include "stdio.h"
#include "task.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//kdkd
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
//#define mainONE_SHOT_TIMER_PERIOD pdMS_TO_TICKS( 3333 )
//#define mainAUTO_RELOAD_TIMER_PERIOD pdMS_TO_TICKS( 500 )
__IO uint32_t OsStatus = 0;
TaskHandle_t task1_handle;
TimerHandle_t xAutoReloadTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

static void prvTimerCallback( TimerHandle_t xTimer );
//static void prvOneShotTimerCallback( TimerHandle_t xTimer );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int i=1000;
void timerCallback(TimerHandle_t xTimer)
{

    printf("Timer expired after %d ms\r\n",i);
    i+=1000;

}*/
const TickType_t xHealthyTimerPeriod = pdMS_TO_TICKS( 3000 );
const TickType_t xErrorTimerPeriod = pdMS_TO_TICKS( 200 );

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /*TimerHandle_t xAutoReloadTimer, xOneShotTimer;
  BaseType_t xTimer1Started, xTimer2Started;

  xOneShotTimer = xTimerCreate("OneShot",mainONE_SHOT_TIMER_PERIOD,pdFALSE,0,prvOneShotTimerCallback );
  xAutoReloadTimer = xTimerCreate("AutoReload",mainAUTO_RELOAD_TIMER_PERIOD,pdTRUE,0,prvAutoReloadTimerCallback );*/
  BaseType_t  xTimer2Started;

    xTaskCreate(StartDefaultTask, "LED Task", 1024, NULL, 1, &task1_handle);

   // xOneShotTimer = xTimerCreate("OneShot",mainONE_SHOT_TIMER_PERIOD,pdFALSE,0,prvTimerCallback );
    xAutoReloadTimer = xTimerCreate("AutoReload",xHealthyTimerPeriod,pdTRUE,0,prvTimerCallback);

    if(  xAutoReloadTimer != NULL  )
     {
      printf("timers created\n");
     //xTimer1Started = xTimerStart( xOneShotTimer, 0 );
     xTimer2Started = xTimerStart( xAutoReloadTimer, 0 );

       if( xTimer2Started == pdPASS  )
        {
           /* Start the scheduler. */
           vTaskStartScheduler();
        }
     }


  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, gled_Pin|r_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : gled_Pin r_led_Pin */
  GPIO_InitStruct.Pin = gled_Pin|r_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */


//	    TimerHandle_t xTimer;
//
//	    // Create a timer with a period of 1000ms (1 second)
//	    xTimer = xTimerCreate("MyTimer", pdMS_TO_TICKS(1000), pdTRUE, 0, timerCallback);
//
//	    if (xTimer == NULL)
//	    {
//	        printf("Failed to create timer!\r\n");
//	        vTaskDelete(NULL);
//	    }
//
//	    // Start the timer
//	    xTimerStart(xTimer, 0);
//
//	    // Delay for 5000ms (5 seconds)
//	    vTaskDelay(pdMS_TO_TICKS(5000));
//	    // Reset the timer
//	        xTimerReset(xTimer, 0);
//
//	        // Delay for another 5000ms (5 seconds)
//	        vTaskDelay(pdMS_TO_TICKS(5000));
//
//	        // Stop and delete the timer
//	        xTimerStop(xTimer, 0);
//	        xTimerDelete(xTimer, 0);
//
//	        // Delete the task
//	        vTaskDelete(NULL);
printf("Task is running\r\n");
while(1)
{
	HAL_GPIO_TogglePin(r_led_GPIO_Port, r_led_Pin);
	HAL_Delay(2000);
}
osDelay(1000);
}

static void prvTimerCallback( TimerHandle_t xTimer )
{

static BaseType_t xErrorDetected = pdFALSE;

if( xErrorDetected == pdFALSE )
    {

    //if( CheckTasksAreRunningWithoutError() == pdFAIL )
        if(eTaskGetState(task1_handle) != eRunning )
        {

            xTimerChangePeriod( xTimer, xErrorTimerPeriod, 0 ); /* Do not block when sending this command. */
        }
        /* Latch that an error has already been detected. */
        xErrorDetected = pdTRUE;
    }
HAL_GPIO_TogglePin(r_led_GPIO_Port, r_led_Pin);
}
  /* USER CODE END 5 */


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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
