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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARP_MEMORY_SIZE 1024
#define PACKET_SIZE 1536
#define POOL_SIZE ((sizeof(NX_PACKET) + PACKET_SIZE) * 16)
#define IP_STACK_SIZE 2048
#define IP_PRIORITY 1
#define HTTP_SERVER_STACK_SIZE 12288
#define HTTP_SERVER_PORT 80
#define ADC_STACK_SIZE 1024
#define ARP_STACK_SIZE 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
NX_PACKET_POOL packet_pool;
NX_IP ip;
NX_TCP_SOCKET tcp_server_socket;

UCHAR ip_stack[IP_STACK_SIZE];
UCHAR packet_pool_memory[POOL_SIZE];
UCHAR arp_memory[ARP_MEMORY_SIZE];

TX_THREAD http_thread;
UCHAR http_stack[HTTP_SERVER_STACK_SIZE];

TX_THREAD adc_thread;
UCHAR adc_stack[ADC_STACK_SIZE];

TX_THREAD arp_thread;
UCHAR arp_stack[ARP_STACK_SIZE];

extern ULONG IP_ADDR;
extern ULONG IP_MASK;
extern volatile float mcu_temperature;
extern volatile float vdd_voltage;
extern ADC_HandleTypeDef hadc1;
extern uint32_t adc_data[];
extern uint8_t adc_flag;
extern ETH_HandleTypeDef heth;

uint8_t flag_change_ip_and_mask = 0;
ULONG ip_addr, mask_addr;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void http_thread_entry(ULONG thread_input);
VOID parse_query(CHAR *query, CHAR *ip, CHAR *mask, CHAR *pass);
VOID send_http_response(NX_TCP_SOCKET *socket, CHAR *data);
UINT ip_string_to_ulong(const CHAR *ip_str, ULONG *ip_addr);
void adc_thread_entry(ULONG thread_input);
void arp_thread_entry(ULONG thread_input);
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
	UINT status;
	uint8_t tx_buffer[64];
	// Create a packet pool
	status = nx_packet_pool_create(&packet_pool, "Packet Pool", PACKET_SIZE,
			packet_pool_memory, POOL_SIZE);
	if (status == NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "Packet pool success\n"),
				HAL_MAX_DELAY);
	}
	// Create an IP instance
	status = nx_ip_create(&ip, "IP Instance", IP_ADDR, IP_MASK, &packet_pool,
			nx_stm32_eth_driver, ip_stack, IP_STACK_SIZE, IP_PRIORITY);
	if (status == NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "IP success\n"), HAL_MAX_DELAY);
	}
	// Start the IP thread
	status = nx_arp_enable(&ip, (void*) arp_memory, ARP_MEMORY_SIZE);
	if (status == NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "ARP success\n"), HAL_MAX_DELAY);
	}
	// Enable ICMP (for ping)
	status = nx_icmp_enable(&ip);
	if (status == NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "ICMP success\n"),
				HAL_MAX_DELAY);
	}
	status = nx_tcp_enable(&ip);
	if (status == NX_SUCCESS) {
		HAL_UART_Transmit(&huart6, tx_buffer,
				sprintf((char*) tx_buffer, "TCP success\n"), HAL_MAX_DELAY);
	}

	tx_thread_create(&http_thread, "HTTP thread", http_thread_entry, 0,
			http_stack, HTTP_SERVER_STACK_SIZE, 5, 5, TX_NO_TIME_SLICE,
			TX_AUTO_START);
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "HTTP thread create\n"),
			HAL_MAX_DELAY);
	tx_thread_create(&adc_thread, "ADC thread", adc_thread_entry, 0, adc_stack,
			ADC_STACK_SIZE, 3, 3, TX_NO_TIME_SLICE, TX_AUTO_START);
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "ADC thread create\n"),
			HAL_MAX_DELAY);
	tx_thread_create(&arp_thread, "ARP thread", arp_thread_entry, 0, arp_stack,
			ARP_STACK_SIZE, 4, 4, TX_NO_TIME_SLICE, TX_AUTO_START);
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "ARP thread create\n"),
			HAL_MAX_DELAY);
	/* USER CODE END MX_NetXDuo_Init */

	return ret;
}

/* USER CODE BEGIN 1 */
void http_thread_entry(ULONG thread_input) {
	CHAR dynamic_page[2048];
	uint8_t flag = 0;
	uint8_t tx_buffer[64];

	// Создание TCP сокета
	nx_tcp_socket_create(&ip, &tcp_server_socket, "HTTP Server Socket",
			NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 512, NX_NULL,
			NX_NULL);

	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "HTTP Server Socket create\n"),
			HAL_MAX_DELAY);

	// Слушаем порт 80
	nx_tcp_server_socket_listen(&ip, HTTP_SERVER_PORT, &tcp_server_socket, 5,
	NX_NULL);

	while (1) {
		if (flag == 1) {
			nx_ip_interface_address_set(&ip, 0, ip_addr, mask_addr);
			flag = 0;
		}

		nx_tcp_server_socket_accept(&tcp_server_socket, TX_WAIT_FOREVER);

		NX_PACKET *packet;
		if (nx_tcp_socket_receive(&tcp_server_socket, &packet,
		TX_WAIT_FOREVER) == NX_SUCCESS) {
			CHAR *data = (CHAR*) packet->nx_packet_prepend_ptr;
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer,
							"Incoming reques to the http server\n"),
					HAL_MAX_DELAY);
			// AJAX-запрос на получение данных
			if (strncmp(data, "GET /new_data", 13) == 0) {
				int whole_temp = (int) mcu_temperature;
				int frac_temp = (int) ((mcu_temperature - whole_temp) * 1000);
				int whole_voltage = (int) vdd_voltage;
				int frac_voltage = (int) ((vdd_voltage - whole_voltage) * 1000);

				char data[128];
				sprintf(data, "Temperature: %d.%03d°C<br>Voltage: %d.%03dV",
						whole_temp, frac_temp, whole_voltage, frac_voltage);
				HAL_UART_Transmit(&huart6, tx_buffer,
						sprintf((char*) tx_buffer,
								"Voltage and temperature updated\n"),
						HAL_MAX_DELAY);
				send_http_response(&tcp_server_socket,
						"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n");
				send_http_response(&tcp_server_socket, data);
			}

			// Обработка формы изменения IP/маски
			else if (strncmp(data, "GET /setip?", 11) == 0) {
				HAL_UART_Transmit(&huart6, tx_buffer,
						sprintf((char*) tx_buffer,
								"Request for a shift ip address and subnet mask\n"),
						HAL_MAX_DELAY);
				CHAR ip_str[16] = { 0 }, mask_str[16] = { 0 }, pass[16] = { 0 };
				parse_query(data + 11, ip_str, mask_str, pass);

				if (strcmp(pass, "admin") == 0) {
					HAL_UART_Transmit(&huart6, tx_buffer,
							sprintf((char*) tx_buffer,
									"The password successfuly\n"),
							HAL_MAX_DELAY);
					if (ip_string_to_ulong(ip_str, &ip_addr) == NX_SUCCESS
							&& ip_string_to_ulong(mask_str, &mask_addr)
									== NX_SUCCESS) {
						HAL_UART_Transmit(&huart6, tx_buffer,
								sprintf((char*) tx_buffer,
										"IP address and subnet mask is valid\n"),
								HAL_MAX_DELAY);
						send_http_response(&tcp_server_socket,
								"HTTP/1.1 200 OK\r\n\r\nIP and subnet changed.");
						flag = 1;
						flag_change_ip_and_mask = 1;
					} else {
						HAL_UART_Transmit(&huart6, tx_buffer,
								sprintf((char*) tx_buffer,
										"IP address and subnet mask is not valid\n"),
								HAL_MAX_DELAY);
						send_http_response(&tcp_server_socket,
								"HTTP/1.1 400 Bad Request\r\n\r\nInvalid IP or mask.");
					}
				} else {
					HAL_UART_Transmit(&huart6, tx_buffer,
							sprintf((char*) tx_buffer,
									"The password not successfuly\n"),
							HAL_MAX_DELAY);
					send_http_response(&tcp_server_socket,
							"HTTP/1.1 403 Forbidden\r\n\r\nIncorrect password.");
				}
			}

			// Главная страница с HTML и JavaScript
			else {
				HAL_UART_Transmit(&huart6, tx_buffer,
						sprintf((char*) tx_buffer, "Default request\n"),
						HAL_MAX_DELAY);
				int whole_temp = (int) mcu_temperature;
				int frac_temp = (int) ((mcu_temperature - whole_temp) * 1000);
				int whole_voltage = (int) vdd_voltage;
				int frac_voltage = (int) ((vdd_voltage - whole_voltage) * 1000);

				sprintf(dynamic_page,
						"HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n"
								"<html><head><title>Radar MMS Project</title>"
								"<style>"
								"body { font-family: Arial; background-color: #f4f4f4; }"
								".top-left { position: absolute; top: 10px; left: 10px; font-size: 24px; font-weight: bold; }"
								".bottom-left { position: absolute; bottom: 20px; left: 10px; }"
								"input { margin: 5px 0; }"
								"</style>"
								"<script>"
								"function fetchData() {"
								"  fetch('/new_data').then(r => r.text()).then(t => { document.getElementById('live').innerHTML = t; });"
								"}"
								"setInterval(fetchData, 1000);"
								"</script></head><body>"

								"<div id='live' class='top-left'>"
								"Temperature: %d.%03d<br>"
								"Voltage: %d.%03d"
								"</div>"

								"<div class='bottom-left'>"
								"<h3>Settings IP and subnet mask</h3>"
								"<form method='GET' action='/setip'>"
								"IP-address:<br><input type='text' name='ip'><br>"
								"Subnet mask:<br><input type='text' name='mask'><br>"
								"Password:<br><input type='password' name='pass'><br>"
								"<input type='submit' value='apply'>"
								"</form>"
								"</div>"

								"</body></html>", whole_temp, frac_temp,
						whole_voltage, frac_voltage);

				send_http_response(&tcp_server_socket, dynamic_page);
			}

			nx_packet_release(packet);
		}
		nx_tcp_socket_disconnect(&tcp_server_socket, TX_WAIT_FOREVER);
		HAL_UART_Transmit(&huart6, (const uint8_t*) tx_buffer,
				sprintf((char*) tx_buffer, "Socket disconnected\n"),
				HAL_MAX_DELAY);
		nx_tcp_server_socket_unaccept(&tcp_server_socket);
		nx_tcp_server_socket_relisten(&ip, 80, &tcp_server_socket);
	}
}

VOID parse_query(CHAR *query, CHAR *ip, CHAR *mask, CHAR *pass) {
	CHAR *ptr;
	if ((ptr = strstr(query, "ip="))) {
		sscanf(ptr, "ip=%15[^&]", ip);
	}
	if ((ptr = strstr(query, "mask="))) {
		sscanf(ptr, "mask=%15[^&]", mask);
	}
	if ((ptr = strstr(query, "pass="))) {
		sscanf(ptr, "pass=%5[^&]", pass);
	}
	uint8_t tx_buffer[64];
	HAL_UART_Transmit(&huart6, (const uint8_t*) tx_buffer,
			sprintf((char*) tx_buffer, "Parsing GET request\n"),
			HAL_MAX_DELAY);
}

VOID send_http_response(NX_TCP_SOCKET *socket, CHAR *data) {
	uint8_t tx_buffer[64];
	HAL_UART_Transmit(&huart6, (const uint8_t*) tx_buffer,
			sprintf((char*) tx_buffer, "Send HTML page\n"),
			HAL_MAX_DELAY);
	NX_PACKET *packet;
	if (nx_packet_allocate(&packet_pool, &packet, NX_TCP_PACKET,
	TX_WAIT_FOREVER) != NX_SUCCESS)
		return;
	nx_packet_data_append(packet, data, strlen(data), &packet_pool,
	TX_WAIT_FOREVER);
	nx_tcp_socket_send(socket, packet, TX_WAIT_FOREVER);
}

UINT ip_string_to_ulong(const CHAR *ip_str, ULONG *ip_addr) {
	UINT a, b, c, d;

	if (sscanf(ip_str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4)
		return NX_INVALID_PARAMETERS;

	if (a > 255 || b > 255 || c > 255 || d > 255)
		return NX_INVALID_PARAMETERS;

	*ip_addr = (a << 24) | (b << 16) | (c << 8) | d;
	return NX_SUCCESS;
}

void adc_thread_entry(ULONG thread_input) {
	while (1) {
		if (adc_flag == 1) {
			HAL_ADC_Start_DMA(&hadc1, adc_data, 2);
			adc_flag = 0;
		}
		tx_thread_sleep(100);
	}
}

void arp_thread_entry(ULONG thread_input) {
	uint32_t reg_value;
	uint8_t tx_buffer[64];
	HAL_StatusTypeDef result;
	while (1) {
		result = HAL_ETH_ReadPHYRegister(&heth, 1, 0x01, &reg_value);
		if (result == HAL_OK && reg_value != 0xFFFF && reg_value != 0x0000) {
			int linked_state = (reg_value & (1 << 2)) >> 2;
			if (linked_state) {
				HAL_UART_Transmit(&huart6, tx_buffer,
						sprintf((char* )tx_buffer, "Valid link established\n"),
						HAL_MAX_DELAY);
				nx_arp_gratuitous_send(&ip, NX_NULL);
				HAL_UART_Transmit(&huart6, tx_buffer,
										sprintf((char* )tx_buffer, "ARP request sended\n"),
										HAL_MAX_DELAY);
			} else {
				HAL_UART_Transmit(&huart6, tx_buffer,
						sprintf((char* )tx_buffer, "Link not established\n"),
						HAL_MAX_DELAY);
			}
		} else {
			HAL_UART_Transmit(&huart6, tx_buffer,
					sprintf((char*) tx_buffer, "PHY address not found\n"),
					HAL_MAX_DELAY);
		}
		tx_thread_sleep(100);
	}
}
/* USER CODE END 1 */
