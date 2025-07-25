/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_STACK_SIZE 1024
#define MEM_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD heartbeat_thread;
UCHAR heartbeat_stack[HEARTBEAT_STACK_SIZE];

TX_THREAD mem_thread;
UCHAR mem_stack[MEM_STACK_SIZE];

extern IWDG_HandleTypeDef hiwdg;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t flag_change_ip_and_mask;
extern ULONG ip_addr;
extern ULONG mask_addr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void heartbeat_entry(ULONG thread_input);
void mem_entry(ULONG thread_input);

extern void write_ip_and_mask(ULONG ip_addr, ULONG mask_addr);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN App_ThreadX_Init */
	(void) byte_pool;
	uint8_t tx_buffer[64];
	tx_thread_create(&heartbeat_thread, "Heartbeat thread", heartbeat_entry, 0,
			heartbeat_stack, HEARTBEAT_STACK_SIZE, 1, 1, TX_NO_TIME_SLICE,
			TX_AUTO_START);
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "Heartbeat thread create\n"),
			HAL_MAX_DELAY);
	tx_thread_create(&mem_thread, "Mem thread", mem_entry, 0, mem_stack,
			MEM_STACK_SIZE, 2, 2, TX_NO_TIME_SLICE, TX_AUTO_START);
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "Mem thread create\n"),
			HAL_MAX_DELAY);
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

/**
  * @brief  MX_ThreadX_Init
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
void heartbeat_entry(ULONG thread_input) {
	while (1) {
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
		HAL_IWDG_Refresh(&hiwdg);
		uint8_t tx_buffer[64];
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer,
						"Reset watchdog timer and led blink\n"),
				HAL_MAX_DELAY);
		tx_thread_sleep(100);
	}
}

void mem_entry(ULONG thread_input) {
	while (1) {
		if (flag_change_ip_and_mask == 1) {
			write_ip_and_mask(ip_addr, mask_addr);
			flag_change_ip_and_mask = 0;
			uint8_t tx_buffer[64];
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "IP address and subnet mask change\n"),
					HAL_MAX_DELAY);
		}
		tx_thread_sleep(50);
	}
}
/* USER CODE END 1 */
