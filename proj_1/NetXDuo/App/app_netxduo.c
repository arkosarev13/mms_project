/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_netxduo.c
 * @author  MCD Application Team
 * @brief   NetXDuo applicative file
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
#include "app_netxduo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nx_stm32_eth_driver.h"
#include "main.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IP_ADDR IP_ADDRESS(173, 159, 125, 100)
#define IP_MASK IP_ADDRESS(255, 255, 255, 0)
#define GATEWAY IP_ADDRESS(173, 159, 125, 0)

#define ARP_MEMORY_SIZE 1024
#define PACKET_SIZE 1536
#define POOL_SIZE ((sizeof(NX_PACKET) + PACKET_SIZE) * 16)
#define IP_STACK_SIZE 2048
#define IP_PRIORITY 1
#define PHY_REG_THREAD_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD phy_reg_thread;
NX_PACKET_POOL packet_pool;
NX_IP ip;

UCHAR ip_stack[IP_STACK_SIZE];
UCHAR packet_pool_memory[POOL_SIZE];
UCHAR arp_memory[ARP_MEMORY_SIZE];
UCHAR phy_reg_thread_stack[PHY_REG_THREAD_STACK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void thread_0_entry(ULONG thread_input);
/* USER CODE END PFP */
/**
 * @brief  Application NetXDuo Initialization.
 * @param memory_ptr: memory pointer
 * @retval int
 */
UINT MX_NetXDuo_Init(VOID *memory_ptr) {
	UINT ret = NX_SUCCESS;
	TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*) memory_ptr;

	/* USER CODE BEGIN MX_NetXDuo_MEM_POOL */
	(void) byte_pool;
	/* USER CODE END MX_NetXDuo_MEM_POOL */

	/* USER CODE BEGIN MX_NetXDuo_Init */
	nx_system_initialize();

	// Create a packet pool
	nx_packet_pool_create(&packet_pool, "Packet Pool", PACKET_SIZE,
			packet_pool_memory, POOL_SIZE);

	// Create an IP instance
	nx_ip_create(&ip, "IP Instance", IP_ADDR, IP_MASK, &packet_pool,
			nx_stm32_eth_driver, ip_stack, IP_STACK_SIZE, IP_PRIORITY);

	// Start the IP thread
	nx_arp_enable(&ip, (void*) arp_memory, ARP_MEMORY_SIZE);
	// Enable ICMP (for ping)
	nx_icmp_enable(&ip);
	nx_tcp_enable(&ip);

	tx_thread_create(&phy_reg_thread, "PHY reg Thread", thread_0_entry, 0,
			phy_reg_thread_stack, PHY_REG_THREAD_STACK_SIZE, 2, 2,
			TX_NO_TIME_SLICE, TX_AUTO_START);
	/* USER CODE END MX_NetXDuo_Init */

	return ret;
}

/* USER CODE BEGIN 1 */
void thread_0_entry(ULONG thread_input) {
	uint32_t reg_value;
	uint8_t tx_buffer[256];
	uint8_t flag = 1;
	HAL_StatusTypeDef result;
	while (1) {
		result = HAL_ETH_ReadPHYRegister(&heth, 1, 0x01, &reg_value);
		if (result == HAL_OK && reg_value != 0xFFFF && reg_value != 0x0000) {
			//int linked_value = (reg_value & (1 << 2)) >> 2;
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer,
							"PHY found at address: %d. BMSR: 0x%04lX\n", 1,
							reg_value), HAL_MAX_DELAY);
			flag = 0;
		}
		result = HAL_ETH_ReadPHYRegister(&heth, 1, 0x00, &reg_value);
		if (result == HAL_OK && reg_value != 0xFFFF && reg_value != 0x0000) {
			//int linked_value = (reg_value & (1 << 2)) >> 2;
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer,
							"PHY found at address: %d. BMCR: 0x%04lX\n", 1,
							reg_value), HAL_MAX_DELAY);
			flag = 0;
		}
		if (flag) {
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "PHY address not found"),
					HAL_MAX_DELAY);
		}
		tx_thread_sleep(100);
	}
}
/* USER CODE END 1 */
