#ifndef INC_MEM_H_
#define INC_MEM_H_

#include "main.h"

#define EEPROM_ADDR 0xA0  // или 0x50 << 1
#define CONFIGURATION_VALUE 1 //если 0, то при перезугразке микроконтроллера ip 192.168.0.13, subnet mask 255.255.255.0, если 1, то значение ip и subnet mask берется из памяти микроконтроллера

HAL_StatusTypeDef EEPROM_WriteByte(uint8_t, uint8_t);
HAL_StatusTypeDef EEPROM_ReadByte(uint8_t, uint8_t*);
void write_ip_and_mask(ULONG, ULONG);
void read_ip_and_mask(ULONG*, ULONG*);
void ip_and_mask();

#endif /* INC_MEM_H_ */
