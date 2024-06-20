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

#define DEBOUNCE_TIME 400 // Debounce time in milliseconds

// Flags to track the state of each button
volatile uint32_t last_interrupt_time_pump1_start = 0;
volatile uint32_t last_interrupt_time_pump1_stop = 0;
volatile uint32_t last_interrupt_time_pump2_start = 0;
volatile uint32_t last_interrupt_time_pump2_stop = 0;

int totalPump = 50000;
int nop = 5; //100 can
int pump1Count = 0;
int pump2Count = 0;
static bool pump1flag = false;
static bool pump2flag = false;
static bool OnPetrolflag = true;
uint32_t timer_start = 0;
uint32_t timer_end = 0;
uint32_t load_value = 0;
int z = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim10;

/* Definitions for pump1_toggle */
osThreadId_t pump1_toggleHandle;
const osThreadAttr_t pump1_toggle_attributes = {
  .name = "pump1_toggle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pump2_toggle */
osThreadId_t pump2_toggleHandle;
const osThreadAttr_t pump2_toggle_attributes = {
  .name = "pump2_toggle",
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
/* Definitions for pump1 */
osSemaphoreId_t pump1Handle;
const osSemaphoreAttr_t pump1_attributes = {
  .name = "pump1"
};
/* Definitions for pump2 */
osSemaphoreId_t pump2Handle;
const osSemaphoreAttr_t pump2_attributes = {
  .name = "pump2"
};
/* USER CODE BEGIN PV */
osEventFlagsId_t PumpGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM10_Init(void);
void pump1toggle(void *argument);
void pump2toggle(void *argument);
void start_lcd(void *argument);

/* USER CODE BEGIN PFP */
void Task_action(char message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();

    // External button stop
    if (GPIO_Pin == stop1_Pin) {
        OnPetrolflag = false;
    }
    // pump 1 start
    else if (GPIO_Pin == pump1_start_Pin) {
        if ((current_time - last_interrupt_time_pump1_start) >= DEBOUNCE_TIME) {
        	timer_start = SysTick -> VAL;
            last_interrupt_time_pump1_start = current_time;
            pump1flag = true;
        }
    }
    // pump 1 stop
    else if (GPIO_Pin == pump1_stop_Pin) {
        if ((current_time - last_interrupt_time_pump1_stop) >= DEBOUNCE_TIME) {
            last_interrupt_time_pump1_stop = current_time;
            pump1flag = false;
        }
    }
    // pump 2 start
    else if (GPIO_Pin == pump2_start_Pin) {
        if ((current_time - last_interrupt_time_pump2_start) >= DEBOUNCE_TIME) {
            last_interrupt_time_pump2_start = current_time;
            pump2flag = true;
        }
    }
    // pump 2 stop
    else if (GPIO_Pin == pump2_stop_Pin) {
        if ((current_time - last_interrupt_time_pump2_stop) >= DEBOUNCE_TIME) {
            last_interrupt_time_pump2_stop = current_time;
            pump2flag = false;
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
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of pump1 */
  pump1Handle = osSemaphoreNew(1, 1, &pump1_attributes);

  /* creation of pump2 */
  pump2Handle = osSemaphoreNew(1, 0, &pump2_attributes);

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
  /* creation of pump1_toggle */
  pump1_toggleHandle = osThreadNew(pump1toggle, NULL, &pump1_toggle_attributes);

  /* creation of pump2_toggle */
  pump2_toggleHandle = osThreadNew(pump2toggle, NULL, &pump2_toggle_attributes);

  /* creation of startLcd */
  startLcdHandle = osThreadNew(start_lcd, NULL, &startLcd_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  const osEventFlagsAttr_t Flags_attr = {.name = "Triggering_PumpThread"};
  PumpGroup = osEventFlagsNew(&Flags_attr);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 1000;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, pump2_Pin|pump1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : pump1_start_Pin pump1_stop_Pin */
  GPIO_InitStruct.Pin = pump1_start_Pin|pump1_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : pump2_start_Pin */
  GPIO_InitStruct.Pin = pump2_start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pump2_start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : pump2_stop_Pin */
  GPIO_InitStruct.Pin = pump2_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(pump2_stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : stop1_Pin */
  GPIO_InitStruct.Pin = stop1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(stop1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : pump2_Pin pump1_Pin */
  GPIO_InitStruct.Pin = pump2_Pin|pump1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0); //highest priority level
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

/* USER CODE BEGIN Header_pump1toggle */
/**
  * @brief  Function implementing the pump1_toggle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_pump1toggle */
void pump1toggle(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	osSemaphoreAcquire(pump1Handle, osWaitForever );
	if(pump1flag && OnPetrolflag && totalPump>0){
//		for(int i = 0 ; i<20; i++){
			HAL_GPIO_WritePin(pump1_GPIO_Port, pump1_Pin, GPIO_PIN_SET);
			for(int i = 0; i<nop; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(pump1_GPIO_Port, pump1_Pin, GPIO_PIN_RESET);
		while(OnPetrolflag){
			pump1Count ++;
			totalPump--;
			break;
		}
	  timer_end = SysTick -> VAL;
	  z++;
//		}
	}
	osSemaphoreRelease(pump2Handle);
	//osThreadYield();
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_pump2toggle */
/**
* @brief Function implementing the pump2_toggle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pump2toggle */
void pump2toggle(void *argument)
{
  /* USER CODE BEGIN pump2toggle */
  /* Infinite loop */
  for(;;)
  {

	osSemaphoreAcquire(pump2Handle, osWaitForever);
	if(pump2flag && OnPetrolflag && totalPump>0){
//		for(int i = 0; i<20; i++){
			//read falling edge
			HAL_GPIO_WritePin(pump2_GPIO_Port, pump2_Pin, GPIO_PIN_SET);
			for(int i = 0; i<nop; i++){
				__NOP();
			}
			HAL_GPIO_WritePin(pump2_GPIO_Port, pump2_Pin, GPIO_PIN_RESET);
		while(OnPetrolflag){
			totalPump--;
			pump2Count ++;
			break;
		}
			//Task_action('2');
//		}
	}
	osSemaphoreRelease(pump1Handle);
	//osThreadYield();
  }
  /* USER CODE END pump2toggle */
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
	//const TickType_t xFrequency = 30; // 50 ticks = 50ms if configTICK_RATE_HZ is 1000.

	HD44780_Init(2);
	HD44780_Clear();
	char Pump1Str[20];
	char Pump2Str[20];
    // Print static parts once
    HD44780_SetCursor(0, 0);
    HD44780_PrintStr("Pump1:");

    HD44780_SetCursor(0, 1);
    HD44780_PrintStr("Pump2:");
  /* USER CODE BEGIN start_lcd */
  /* Infinite loop */
  for(;;)
  {

	  // Wait for the next cycle
	  //vTaskDelayUntil(&xLastWakeTime, xFrequency); // this thread will run every 50uwtick
      // Print Pump1 count
      HD44780_SetCursor(6, 0);
      snprintf(Pump1Str, sizeof(Pump1Str), " %7d", pump1Count);
      HD44780_PrintStr(Pump1Str);

      // Print Pump2 count
      HD44780_SetCursor(6, 1);
      snprintf(Pump2Str, sizeof(Pump2Str), " %7d", pump2Count);
      HD44780_PrintStr(Pump2Str);
      //osThreadYield();
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
