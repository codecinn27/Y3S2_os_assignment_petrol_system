/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "liquidcrystal_i2c.h"
#include <stdint.h>  // Include for fixed-width integer types
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 400 //milliseconds debounce delay

// Flags to track the state of each button
volatile uint32_t last_press_time_pump3_start = 0;
volatile uint32_t last_press_time_pump3_stop = 0;
volatile uint32_t last_press_time_pump4_start = 0;
volatile uint32_t last_press_time_pump4_stop = 0;

int totalPump = 50000;
int nop = 5;
int pump3Count = 0;
int pump4Count = 0;
static bool pump3flag = false;
static bool pump4flag = false;
static bool OnPetrolflag = true;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Definitions for pump3_toggle */
osThreadId_t pump3_toggleHandle;
const osThreadAttr_t pump3_toggle_attributes = {
  .name = "pump3_toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pump4_toggle */
osThreadId_t pump4_toggleHandle;
const osThreadAttr_t pump4_toggle_attributes = {
  .name = "pump4_toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for startLcd */
osThreadId_t startLcdHandle;
const osThreadAttr_t startLcd_attributes = {
  .name = "startLcd",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pump3 */
osSemaphoreId_t pump3Handle;
const osSemaphoreAttr_t pump3_attributes = {
  .name = "pump3"
};
/* Definitions for pump4 */
osSemaphoreId_t pump4Handle;
const osSemaphoreAttr_t pump4_attributes = {
  .name = "pump4"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void pump3toggle(void *argument);
void pump4toggle(void *argument);
void start_lcd(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(char message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t current_time = HAL_GetTick();
    // External button, off
    if(GPIO_Pin == stop2_Pin){
    	OnPetrolflag = false;
    }
    // pump 3 start
    else if (GPIO_Pin == pump3_start_Pin) {
    	if((current_time - last_press_time_pump3_start) >= DEBOUNCE_DELAY){
    		last_press_time_pump3_start = current_time;
    		pump3flag = true;
    	}
    }
    //pump 3 stop
    else if (GPIO_Pin == pump3_stop_Pin) {
    	if((current_time - last_press_time_pump3_stop) >= DEBOUNCE_DELAY){
    		last_press_time_pump3_stop = current_time;
    		pump3flag = false;
    	}

    }
    else if (GPIO_Pin == pump4_start_Pin) {
    	if((current_time - last_press_time_pump4_start) >= DEBOUNCE_DELAY){
    		last_press_time_pump4_start = current_time;
    		pump4flag = true;
    	}
    }
    else if (GPIO_Pin == pump4_stop_Pin) {
    	if((current_time - last_press_time_pump4_stop) >= DEBOUNCE_DELAY){
    		last_press_time_pump4_stop = current_time;
    		pump4flag = false;
    	}
    }
}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of pump3 */
  pump3Handle = osSemaphoreNew(1, 1, &pump3_attributes);

  /* creation of pump4 */
  pump4Handle = osSemaphoreNew(1, 0, &pump4_attributes);

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
  /* creation of pump3_toggle */
  pump3_toggleHandle = osThreadNew(pump3toggle, NULL, &pump3_toggle_attributes);

  /* creation of pump4_toggle */
  pump4_toggleHandle = osThreadNew(pump4toggle, NULL, &pump4_toggle_attributes);

  /* creation of startLcd */
  startLcdHandle = osThreadNew(start_lcd, NULL, &startLcd_attributes);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pump4_GPIO_Port, pump4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(pump3_GPIO_Port, pump3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : stop2_Pin */
  GPIO_InitStruct.Pin = stop2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(stop2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pump4_Pin */
  GPIO_InitStruct.Pin = pump4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pump4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pump3_Pin */
  GPIO_InitStruct.Pin = pump3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(pump3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump4_start_Pin pump4_stop_Pin */
  GPIO_InitStruct.Pin = pump4_start_Pin|pump4_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : pump3_start_Pin pump3_stop_Pin */
  GPIO_InitStruct.Pin = pump3_start_Pin|pump3_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Task_action(char message)
{
	ITM_SendChar(message);
	ITM_SendChar('\n');
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_pump3toggle */
/**
  * @brief  Function implementing the pump3_toggle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pump3toggle */
void pump3toggle(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	//(pump3Handle, osWaitForever);
	osSemaphoreAcquire(pump3Handle, osWaitForever);
	if(pump3flag && OnPetrolflag && totalPump>0){
		//for(int i = 0; i<20; i++){
			//read falling edge
			HAL_GPIO_WritePin(pump3_GPIO_Port, pump3_Pin, GPIO_PIN_SET);
			for(int i = 0; i<nop; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(pump3_GPIO_Port, pump3_Pin, GPIO_PIN_RESET);
			while(OnPetrolflag){
				totalPump--;
				pump3Count ++;
				break;
			}

			//Task_action('3');
		//}
	}
	osSemaphoreRelease(pump4Handle);
	//osThreadYield();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pump4toggle */
/**
* @brief Function implementing the pump4_toggle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pump4toggle */
void pump4toggle(void *argument)
{
  /* USER CODE BEGIN pump4toggle */
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreAcquire(pump4Handle, osWaitForever);
	if(pump4flag && OnPetrolflag && totalPump>0){
		//for(int i=0; i<20; i++){
			//read falling edge
			HAL_GPIO_WritePin(pump4_GPIO_Port, pump4_Pin, GPIO_PIN_SET);
			for(int i = 0; i<nop; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(pump4_GPIO_Port, pump4_Pin, GPIO_PIN_RESET);
			while(OnPetrolflag){
				totalPump--;
				pump4Count ++;
				break;
			}
			//Task_action('4');
		//}
	}
	osSemaphoreRelease(pump3Handle);
	//osThreadYield();
  }
  /* USER CODE END pump4toggle */
}

/* USER CODE BEGIN Header_start_lcd */
/**
* @brief Function implementing the startLcd thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_lcd */
void start_lcd(void *argument)
{
  /* USER CODE BEGIN start_lcd */
	//TickType_t xLastWakeTime = xTaskGetTickCount(); // Initialize the xLastWakeTime variable with the current time.
	//const TickType_t xFrequency = 50; // 50 ticks = 50ms if configTICK_RATE_HZ is 1000.

	HD44780_Init(2);
	HD44780_Clear();
	char Pump3Str[20];
	char Pump4Str[20];
    // Print static parts once
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Pump3:");

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("Pump4:");
  /* Infinite loop */
  for(;;)
  {
	  // Wait for the next cycle
	 // vTaskDelayUntil(&xLastWakeTime, xFrequency); // this thread will run every 50uwtick
      HD44780_SetCursor(6, 0);
      snprintf(Pump3Str, sizeof(Pump3Str), " %7d", pump3Count);
      HD44780_PrintStr(Pump3Str);

      // Print Pump2 count
      HD44780_SetCursor(6, 1);
      snprintf(Pump4Str, sizeof(Pump4Str), " %7d", pump4Count);
      HD44780_PrintStr(Pump4Str);
  }
  /* USER CODE END start_lcd */
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
