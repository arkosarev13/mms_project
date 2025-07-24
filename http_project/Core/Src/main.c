/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VREFINT_CAL 1.21f
#define ADC_RESOLUTION 4095.0f
#define AVG_SLOPE 0.0025f
#define V25 0.76f
#define EEPROM_ADDR 0xA0  // или 0x50 << 1
#define CONFIGURATION_VALUE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint32_t adc_data[2];
float mcu_temperature;
float vdd_voltage;
uint8_t adc_flag = 0;
ULONG IP_ADDR;
ULONG IP_MASK;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef EEPROM_WriteByte(uint8_t memAddress, uint8_t data) {
	return HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, memAddress,
	I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// Читаем байт из EEPROM по адресу
HAL_StatusTypeDef EEPROM_ReadByte(uint8_t memAddress, uint8_t *data) {
	return HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, memAddress,
	I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

void write_ip_and_mask(ULONG ip_addr, ULONG mask_addr) {
	EEPROM_WriteByte(0x00, ((uint8_t) ((ip_addr & (0xFF << 24)) >> 24)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x01, ((uint8_t) ((ip_addr & (0xFF << 16)) >> 16)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x02, ((uint8_t) ((ip_addr & (0xFF << 8)) >> 8)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x03, ((uint8_t) ((ip_addr & (0xFF << 0)) >> 0)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x04, ((uint8_t) ((mask_addr & (0xFF << 24)) >> 24)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x05, ((uint8_t) ((mask_addr & (0xFF << 16)) >> 16)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x06, ((uint8_t) ((mask_addr & (0xFF << 8)) >> 8)));
	HAL_Delay(10);
	EEPROM_WriteByte(0x07, ((uint8_t) ((mask_addr & (0xFF << 0)) >> 0)));
	HAL_Delay(10);
}

void read_ip_and_mask(ULONG *ip_addr, ULONG *mask_addr) {
	uint8_t data = 0;
	*ip_addr = 0;
	*mask_addr = 0;
	EEPROM_ReadByte(0x00, &data);
	*ip_addr |= ((ULONG) data << 24);
	EEPROM_ReadByte(0x01, &data);
	*ip_addr |= ((ULONG) data << 16);
	EEPROM_ReadByte(0x02, &data);
	*ip_addr |= ((ULONG) data << 8);
	EEPROM_ReadByte(0x03, &data);
	*ip_addr |= ((ULONG) data << 0);
	EEPROM_ReadByte(0x04, &data);
	*mask_addr |= ((ULONG) data << 24);
	EEPROM_ReadByte(0x05, &data);
	*mask_addr |= ((ULONG) data << 16);
	EEPROM_ReadByte(0x06, &data);
	*mask_addr |= ((ULONG) data << 8);
	EEPROM_ReadByte(0x07, &data);
	*mask_addr |= ((ULONG) data << 0);
}

void print_ip_ulong_uart(ULONG ip) {
	char buffer[32];

	uint8_t ip_parts[4];
	ip_parts[0] = (uint8_t) (ip >> 24);       // старший байт
	ip_parts[1] = (uint8_t) (ip >> 16);
	ip_parts[2] = (uint8_t) (ip >> 8);
	ip_parts[3] = (uint8_t) (ip);             // младший байт

	snprintf(buffer, sizeof(buffer), "%u.%u.%u.%u\r\n", ip_parts[0],
			ip_parts[1], ip_parts[2], ip_parts[3]);

	HAL_UART_Transmit(&huart6, (uint8_t*) buffer, strlen(buffer),
	HAL_MAX_DELAY);
}

void ip_and_mask() {
	if (CONFIGURATION_VALUE == 0) {
		IP_ADDR = IP_ADDRESS(192, 168, 0, 13);
		IP_MASK = IP_ADDRESS(255, 255, 255, 0);
		write_ip_and_mask(IP_ADDR, IP_MASK);
		read_ip_and_mask(&IP_ADDR, &IP_MASK);
		print_ip_ulong_uart(IP_ADDR);
		print_ip_ulong_uart(IP_MASK);
	} else {
		read_ip_and_mask(&IP_ADDR, &IP_MASK);
		print_ip_ulong_uart(IP_ADDR);
		print_ip_ulong_uart(IP_MASK);
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint8_t tx_buffer[64];
	uint32_t raw_vref = adc_data[0];
	uint32_t raw_temp = adc_data[1];
	// Вычисление VDD на основе VREFINT
	vdd_voltage = (VREFINT_CAL * ADC_RESOLUTION) / (float) raw_vref;
	HAL_UART_Transmit(&huart6, tx_buffer,
			sprintf((char*) tx_buffer, "Raw vref = %ld. Raw temp = %ld\n",
					raw_vref, raw_temp), HAL_MAX_DELAY);
	// Напряжение на температурном датчике
	float v_sense = (raw_temp * vdd_voltage) / ADC_RESOLUTION;
	// Перевод в температуру (по формуле из даташита)
	mcu_temperature = ((v_sense - V25) / AVG_SLOPE) + 25.0f;
	int whole_voltage = (int) vdd_voltage;
	int frac_voltage = (int) ((vdd_voltage - whole_voltage) * 100); // 2 знака после запятой
	int whole_temp = (int) mcu_temperature;
	int frac_temp = (int) ((mcu_temperature - whole_temp) * 1000); // 2 знака после запятой
	uint8_t tx_buff[64];
	HAL_UART_Transmit(&huart6, (const uint8_t*) tx_buff,
			sprintf((char*) tx_buff,
					"Vdd voltage = %d.%02d\nTemperature = %d.%03d\n",
					whole_voltage, frac_voltage, whole_temp, frac_temp),
			HAL_MAX_DELAY);
	adc_flag = 1;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_ETH_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_I2C1_Init();
	MX_IWDG_Init();
	/* USER CODE BEGIN 2 */
	ip_and_mask();
	HAL_ADC_Start_DMA(&hadc1, adc_data, 2);
	/* USER CODE END 2 */

	MX_ThreadX_Init();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ETH Initialization Function
 * @param None
 * @retval None
 */
static void MX_ETH_Init(void) {

	/* USER CODE BEGIN ETH_Init 0 */

	/* USER CODE END ETH_Init 0 */

	static uint8_t MACAddr[6];

	/* USER CODE BEGIN ETH_Init 1 */

	/* USER CODE END ETH_Init 1 */
	heth.Instance = ETH;
	MACAddr[0] = 0x00;
	MACAddr[1] = 0x80;
	MACAddr[2] = 0xE1;
	MACAddr[3] = 0x00;
	MACAddr[4] = 0x00;
	MACAddr[5] = 0x00;
	heth.Init.MACAddr = &MACAddr[0];
	heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
	heth.Init.TxDesc = DMATxDscrTab;
	heth.Init.RxDesc = DMARxDscrTab;
	heth.Init.RxBuffLen = 1536;

	/* USER CODE BEGIN MACADDRESS */

	/* USER CODE END MACADDRESS */

	if (HAL_ETH_Init(&heth) != HAL_OK) {
		Error_Handler();
	}

	memset(&TxConfig, 0, sizeof(ETH_TxPacketConfig));
	TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM
			| ETH_TX_PACKETS_FEATURES_CRCPAD;
	TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
	TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
	/* USER CODE BEGIN ETH_Init 2 */

	/* USER CODE END ETH_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Reload = 624;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PE13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
