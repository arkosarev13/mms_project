#include "mem.h"

extern I2C_HandleTypeDef hi2c1;
extern ULONG IP_ADDR;
extern ULONG IP_MASK;

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

void ip_and_mask() {
	uint8_t tx_buffer[64];
	HAL_UART_Transmit(&huart6, tx_buffer, sprintf((char* )tx_buffer, "Initialization ip address and subnet mask\n"), HAL_MAX_DELAY);
	if (CONFIGURATION_VALUE == 0) {
		IP_ADDR = IP_ADDRESS(192, 168, 0, 13);
		IP_MASK = IP_ADDRESS(255, 255, 255, 0);
		write_ip_and_mask(IP_ADDR, IP_MASK);
	} else {
		read_ip_and_mask(&IP_ADDR, &IP_MASK);
	}
}
