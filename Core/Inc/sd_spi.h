/*
 * sd_spi.h
 *
 *  Created on: May 11, 2026
 *      Author: aboba
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#pragma once
#include "main.h"

void CS_High(void);
void CS_Low(void);
uint8_t SPI_RW(uint8_t data);
uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg);
uint8_t SD_WaitReady(uint32_t timeout);
int SD_ReadDataBlock(uint8_t *buff, uint32_t btr);
int SD_WriteDataBlock(const uint8_t *buff, uint8_t token);

#endif /* INC_SD_SPI_H_ */
