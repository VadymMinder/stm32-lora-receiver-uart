/*
 * sd_spi.c
 *
 *  Created on: May 11, 2026
 *      Author: aboba
 */


#include "sd_spi.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
#define SD_SPI     &hspi1
#define SD_CS_PORT GPIOB
#define SD_CS_PIN  GPIO_PIN_2

void CS_High(void) { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET); }
void CS_Low(void)  { HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET); }

uint8_t SPI_RW(uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(SD_SPI, &data, &rx, 1, 100);
    return rx;
}

uint8_t SD_WaitReady(uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < timeout) {
        if (SPI_RW(0xFF) == 0xFF) return 1;
    }
    return 0;
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
    if (cmd & 0x80) {
        cmd &= 0x7F;
        uint8_t r = SD_SendCmd(55, 0);
        if (r > 1) return r;
    }

    CS_High(); SPI_RW(0xFF);
    CS_Low();  SPI_RW(0xFF);

    SPI_RW(cmd | 0x40);
    SPI_RW((uint8_t)(arg >> 24));
    SPI_RW((uint8_t)(arg >> 16));
    SPI_RW((uint8_t)(arg >> 8));
    SPI_RW((uint8_t)(arg));

    uint8_t crc = 0xFF;
    if (cmd == 0) crc = 0x95;
    if (cmd == 8) crc = 0x87;
    SPI_RW(crc);

    uint8_t r;
    uint8_t n = 10;
    do { r = SPI_RW(0xFF); } while ((r & 0x80) && --n);
    return r;
}

int SD_ReadDataBlock(uint8_t *buff, uint32_t btr) {
    uint8_t token;
    uint32_t start = HAL_GetTick();
    do { token = SPI_RW(0xFF); } while (token == 0xFF && HAL_GetTick() - start < 200);
    if (token != 0xFE) return 0;

    HAL_SPI_Receive(SD_SPI, buff, btr, 200);
    SPI_RW(0xFF); SPI_RW(0xFF); // CRC
    return 1;
}

int SD_WriteDataBlock(const uint8_t *buff, uint8_t token) {
    if (!SD_WaitReady(500)) return 0;
    SPI_RW(token);
    if (token != 0xFD) {
        HAL_SPI_Transmit(SD_SPI, (uint8_t*)buff, 512, 200);
        SPI_RW(0xFF); SPI_RW(0xFF); // CRC
        uint8_t r = SPI_RW(0xFF) & 0x1F;
        if (r != 0x05) return 0;
    }
    return 1;
}
