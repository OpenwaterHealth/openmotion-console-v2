/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2020-2021 STMicroelectronics.
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
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tx_api.h"
#include "main.h"
#include "led_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_THREAD_STACK_SIZE 1024
#define LED_THREAD_PRIORITY   12

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD led_thread;
UCHAR led_thread_stack[LED_THREAD_STACK_SIZE];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void led_thread_entry(ULONG thread_input);

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */

  // Create the LED thread
  tx_thread_create(&led_thread,               // Thread control block
                   "LED Thread",              // Thread name
                   led_thread_entry,          // Entry function
                   0,                         // Input parameter
                   led_thread_stack,          // Stack start
                   LED_THREAD_STACK_SIZE,     // Stack size
                   LED_THREAD_PRIORITY,       // Priority
                   LED_THREAD_PRIORITY,       // Preemption threshold
                   TX_NO_TIME_SLICE,          // Time slice
                   TX_AUTO_START);            // Auto start
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */


void led_thread_entry(ULONG thread_input)
{
    LED_Init();

    while (1)
    {
    	LED_Toggle(IND1_GPIO_Port, IND1_Pin);
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND / 2); // Sleep for 500ms
    }
}
/* USER CODE END 1 */
