/*
 * sd_logger.c
 *
 *  Created on: May 7, 2026
 *      Author: aboba
 */


#include "sd_logger.h"
#include <stdio.h>
#include <string.h>
#include "structs.h"

extern UART_HandleTypeDef huart1;

static FATFS fs;
static FIL   file;
static uint8_t mounted = 0;
extern char USERPath[4];

SD_LogStatus SD_Logger_Init(void)
{

    FRESULT res;

    res = f_mount(&fs, USERPath, 1);
    char dbg[48];
    snprintf(dbg, sizeof(dbg), "f_mount: %d\r\n", res);
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
    //if (res != FR_OK) return SD_LOG_ERROR;

    if (res == FR_NO_FILESYSTEM) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"Formatting...\r\n", 15, 100);

            BYTE work[512];
            res = f_mkfs(USERPath, FM_FAT32, 0, work, sizeof(work));

            char dbg2[32];
            snprintf(dbg2, sizeof(dbg2), "f_mkfs: %d\r\n", res);
            HAL_UART_Transmit(&huart1, (uint8_t*)dbg2, strlen(dbg2), 100);

            if (res != FR_OK) return SD_LOG_ERROR;

            res = f_mount(&fs, USERPath, 1);  // монтуємо знову
        }

    res = f_open(&file, "log.csv", FA_OPEN_APPEND | FA_WRITE);
    snprintf(dbg, sizeof(dbg), "f_open: %d\r\n", res);
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
    if (res != FR_OK) return SD_LOG_ERROR;

    if (f_size(&file) == 0) {
        f_puts("time_s,temp,pressure,humidity\n", &file);
        HAL_UART_Transmit(&huart1, (uint8_t*)"Header written\r\n", 16, 100);
    }

    mounted = 1;
    HAL_UART_Transmit(&huart1, (uint8_t*)"SD Logger ready\r\n", 17, 100);
    return SD_LOG_OK;
}

SD_LogStatus SD_Logger_Write(const NodeData *d)
{
    if (!mounted) return SD_LOG_ERROR;

    char filename[16];
    snprintf(filename, sizeof(filename), "log_%d.csv", d->id);

    FIL f;
    FRESULT res = f_open(&f, filename, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) return SD_LOG_ERROR;

    if (f_size(&f) == 0) {
        f_puts("temperature,humidity,pressure\n", &f);
    }

    char line[64];
    snprintf(line, sizeof(line), "%d.%02d,%d.%02d,%d.%02d\n",
        (int)d->temp,     (int)((d->temp - (int)d->temp) * 100),
        (int)d->humidity, (int)((d->humidity - (int)d->humidity) * 100),
        (int)d->pressure, (int)((d->pressure - (int)d->pressure) * 100));

    UINT written;
    res = f_write(&f, line, strlen(line), &written);
    f_close(&f);

    return (res == FR_OK) ? SD_LOG_OK : SD_LOG_ERROR;
}

void SD_Logger_Flush(void)
{
    if (mounted) f_sync(&file);  // скидаємо буфер на карту
}
