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
#include "stdio.h"
#include "nx_api.h"
#include "tx_api.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IP_ADDR IP_ADDRESS(192, 168, 0, 13)
#define IP_MASK IP_ADDRESS(255, 255, 255, 0)
#define BROADCAST_IP_ADDRESS  IP_ADDRESS(192,168,0,255)
//#define GATEWAY IP_ADDRESS(173, 159, 125, 0)
#define UDP_PORT 5000

#define ARP_MEMORY_SIZE 1024
#define PACKET_SIZE 1536
#define POOL_SIZE ((sizeof(NX_PACKET) + PACKET_SIZE) * 16)
#define IP_STACK_SIZE 2048
#define IP_PRIORITY 1
#define PHY_REG_THREAD_STACK_SIZE 1024
#define BROADCAST_REQUEST_THREAD_STACK_SIZE 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD phy_reg_thread;
TX_THREAD broadcast_request_thread;
NX_PACKET_POOL packet_pool;
NX_IP ip;
NX_UDP_SOCKET udp_socket;

UCHAR ip_stack[IP_STACK_SIZE];
UCHAR packet_pool_memory[POOL_SIZE];
UCHAR arp_memory[ARP_MEMORY_SIZE];
UCHAR phy_reg_thread_stack[PHY_REG_THREAD_STACK_SIZE];
UCHAR broadcast_request_thread_stack[PHY_REG_THREAD_STACK_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void thread_0_entry(ULONG thread_input);
void broadcast_request(ULONG thread_input);
/* USER CODE END PFP */
/**
  * @brief  Application NetXDuo Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT MX_NetXDuo_Init(VOID *memory_ptr)
{
  UINT ret = NX_SUCCESS;
  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

  /* USER CODE BEGIN MX_NetXDuo_MEM_POOL */
	(void) byte_pool;
  /* USER CODE END MX_NetXDuo_MEM_POOL */

  /* USER CODE BEGIN MX_NetXDuo_Init */
	nx_system_initialize();
	UINT status;
	uint8_t tx_buffer[64];
	// Create a packet pool
	status = nx_packet_pool_create(&packet_pool, "Packet Pool", PACKET_SIZE,
			packet_pool_memory, POOL_SIZE);
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "Packet pool is success\n"),
				HAL_MAX_DELAY);
	}
	// Create an IP instance
	status = nx_ip_create(&ip, "IP Instance", IP_ADDR, IP_MASK, &packet_pool,
			nx_stm32_eth_driver, ip_stack, IP_STACK_SIZE, IP_PRIORITY);
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "IP is success\n"), HAL_MAX_DELAY);
	}
	// Start the IP thread
	status = nx_arp_enable(&ip, (void*) arp_memory, ARP_MEMORY_SIZE);
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "ARP is success\n"), HAL_MAX_DELAY);
	}
	// Enable ICMP (for ping)
	status = nx_icmp_enable(&ip);
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "ICMP is success\n"), HAL_MAX_DELAY);
	}
	status = nx_tcp_enable(&ip);
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "TCP is success\n"), HAL_MAX_DELAY);
	}
	status = nx_udp_enable(&ip);

	/* Check for UDP enable errors.  */
	if (!status) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "UDP is success\n"), HAL_MAX_DELAY);
	}
	tx_thread_create(&phy_reg_thread, "PHY reg Thread", thread_0_entry, 0,
			phy_reg_thread_stack, PHY_REG_THREAD_STACK_SIZE, 2, 2,
			TX_NO_TIME_SLICE, TX_AUTO_START);
	tx_thread_create(&broadcast_request_thread, "Broadcast request thread",
			broadcast_request, 0, broadcast_request_thread_stack,
			BROADCAST_REQUEST_THREAD_STACK_SIZE, 2, 2, TX_NO_TIME_SLICE,
			TX_AUTO_START);
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
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer,
						"Counter TX value = %ld, Counter RX value = %ld\n",
						counter_tx, counter_rx),
				HAL_MAX_DELAY);
		tx_thread_sleep(NX_IP_PERIODIC_RATE);
	}
}

void broadcast_request(ULONG thread_input) {
	UINT status;
	NX_PACKET *packet;
	uint8_t tx_buffer[128];
	// Создаём UDP-сокет
	status = nx_udp_socket_create(&ip, &udp_socket, "UDP Broadcast Socket\n",
			NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 512);
	if (status != NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "Socket not created\n"),
				HAL_MAX_DELAY);
	}
	status = nx_udp_socket_bind(&udp_socket, UDP_PORT, TX_WAIT_FOREVER);
	if (status != NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "UDP socket not binded\n"),
				HAL_MAX_DELAY);
	}
	while (1) {
		status = nx_packet_allocate(&packet_pool, &packet, NX_UDP_PACKET,
		TX_WAIT_FOREVER);
		if (status != NX_SUCCESS) {
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "Packet not allocated\n"),
					HAL_MAX_DELAY);
		}
		// Пишем данные в пакет
		char *msg = "Hello from STM32 (broadcast)!";
		nx_packet_data_append(packet, msg, strlen(msg), &packet_pool,
		TX_WAIT_FOREVER);

		// Отправляем UDP Broadcast
		status = nx_udp_socket_send(&udp_socket, packet, BROADCAST_IP_ADDRESS,
				UDP_PORT);
		if (status != NX_SUCCESS) {
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "Error of send\n"),
					HAL_MAX_DELAY);
			nx_packet_release(packet); // Освобождаем в случае ошибки
		}
		if (status == NX_SUCCESS) {
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "Message sending\n"),
					HAL_MAX_DELAY);
		}
		// Пауза 1 секунда
		tx_thread_sleep(NX_IP_PERIODIC_RATE); // 100 тиков ≈ 1 сек
	}
}
/* USER CODE END 1 */
