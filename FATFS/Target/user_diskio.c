/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   SD card diskio driver via SPI for FatFS
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */
#include <string.h>
#include <stdio.h>
#include "ff_gen_drv.h"
#include "user_diskio.h"
#include "sd_spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;
#define SD_SPI      &hspi1
#define SD_CS_PORT  GPIOB
#define SD_CS_PIN   GPIO_PIN_2

#define CT_MMC   0x01
#define CT_SD1   0x02
#define CT_SD2   0x04
#define CT_BLOCK 0x08

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType = 0;
/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	if (pdrv) return STA_NOINIT;

	    /* Знижуємо швидкість SPI для ініціалізації (~187 кГц) */
	    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	    HAL_SPI_Init(&hspi1);

	    /* 80+ тактів з CS High */
	    CS_High();
	    for (int i = 0; i < 10; i++) SPI_RW(0xFF);

	    /* CMD0 — GO_IDLE_STATE */
	    uint8_t r = SD_SendCmd(0, 0);
	    char dbg[48];
	    snprintf(dbg, sizeof(dbg), "CMD0: 0x%02X\r\n", r);
	    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);

	    if (r != 0x01) {
	        CS_High();
	        return Stat; /* STA_NOINIT */
	    }

	    uint8_t ct = 0;
	    uint32_t start = HAL_GetTick();

	    if (SD_SendCmd(8, 0x1AA) == 1) {
	        /* SDv2 */
	        uint8_t ocr[4];
	        for (int i = 0; i < 4; i++) ocr[i] = SPI_RW(0xFF);

	        if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
	            /* ACMD41 з HCS бітом */
	            while (HAL_GetTick() - start < 2000) {
	                if (SD_SendCmd(0x80 | 41, 0x40000000) == 0) break;
	            }
	            if (HAL_GetTick() - start < 2000 && SD_SendCmd(58, 0) == 0) {
	                for (int i = 0; i < 4; i++) ocr[i] = SPI_RW(0xFF);
	                ct = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
	            }
	        }
	    } else {
	        /* SDv1 або MMC */
	        uint8_t cmd;
	        if (SD_SendCmd(0x80 | 41, 0) <= 1) {
	            ct = CT_SD1; cmd = 0x80 | 41;
	        } else {
	            ct = CT_MMC; cmd = 1;
	        }
	        while (HAL_GetTick() - start < 2000) {
	            if (SD_SendCmd(cmd, 0) == 0) break;
	        }
	        if (HAL_GetTick() - start >= 2000 || SD_SendCmd(16, 512) != 0)
	            ct = 0;
	    }

	    CardType = ct;
	    CS_High(); SPI_RW(0xFF);

	    if (ct) {
	        /* Підвищуємо швидкість після успішної ініціалізації */
	        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	        HAL_SPI_Init(&hspi1);
	        Stat &= ~STA_NOINIT;

	        snprintf(dbg, sizeof(dbg), "CardType: 0x%02X (%s)\r\n", ct,
	            (ct & CT_BLOCK) ? "SDHC/SDXC" :
	            (ct & CT_SD2)   ? "SDv2"       :
	            (ct & CT_SD1)   ? "SDv1"       : "MMC");
	        HAL_UART_Transmit(&huart1, (uint8_t*)dbg, strlen(dbg), 100);
	        HAL_UART_Transmit(&huart1, (uint8_t*)"SD OK\r\n", 7, 100);
	    } else {
	        HAL_UART_Transmit(&huart1, (uint8_t*)"SD FAIL\r\n", 9, 100);
	    }

	    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	if (pdrv) return STA_NOINIT;
	    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (pdrv || !count) return RES_PARERR;
	    if (Stat & STA_NOINIT) return RES_NOTRDY;

	    /* Байтова адресація для не-BLOCK карт */
	    if (!(CardType & CT_BLOCK)) sector *= 512;

	    DRESULT res = RES_ERROR;

	    if (count == 1) {
	        if (SD_SendCmd(17, sector) == 0) {
	            if (SD_ReadDataBlock(buff, 512)) res = RES_OK;
	        }
	    } else {
	        if (SD_SendCmd(18, sector) == 0) {
	            do {
	                if (!SD_ReadDataBlock(buff, 512)) break;
	                buff += 512;
	            } while (--count);
	            SD_SendCmd(12, 0); /* STOP_TRANSMISSION */
	            if (!count) res = RES_OK;
	        }
	    }

	    CS_High(); SPI_RW(0xFF);
	    return res;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	if (pdrv || !count) return RES_PARERR;
	    if (Stat & STA_NOINIT) return RES_NOTRDY;
	    if (Stat & STA_PROTECT) return RES_WRPRT;

	    if (!(CardType & CT_BLOCK)) sector *= 512;

	    DRESULT res = RES_ERROR;

	    if (count == 1) {
	        if (SD_SendCmd(24, sector) == 0) {
	            if (SD_WriteDataBlock(buff, 0xFE)) res = RES_OK;
	        }
	    } else {
	        if (CardType & CT_SD1) SD_SendCmd(0x80 | 23, count); /* ACMD23 */
	        if (SD_SendCmd(25, sector) == 0) {
	            do {
	                if (!SD_WriteDataBlock(buff, 0xFC)) break;
	                buff += 512;
	            } while (--count);
	            if (!SD_WriteDataBlock(0, 0xFD)) count = 1; /* stop token */
	            if (!count) res = RES_OK;
	        }
	    }

	    CS_High(); SPI_RW(0xFF);
	    return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	if (pdrv) return RES_PARERR;
	    if (Stat & STA_NOINIT) return RES_NOTRDY;

	    DRESULT res = RES_ERROR;

	    switch (cmd) {
	        case CTRL_SYNC:
	            CS_Low();
	            if (SD_WaitReady(500)) res = RES_OK;
	            CS_High();
	            break;

	        case GET_SECTOR_COUNT: {
	            uint8_t csd[16];
	            if (SD_SendCmd(9, 0) == 0 && SD_ReadDataBlock(csd, 16)) {
	                DWORD csize;
	                if ((csd[0] >> 6) == 1) {
	                    /* SDv2 */
	                    csize = ((DWORD)(csd[7] & 0x3F) << 16) |
	                            ((DWORD)csd[8] << 8) | csd[9];
	                    *(DWORD*)buff = (csize + 1) << 10;
	                } else {
	                    /* SDv1/MMC */
	                    uint8_t n = (csd[5] & 15) +
	                                ((csd[10] & 128) >> 7) +
	                                ((csd[9] & 3) << 1) + 2;
	                    csize = ((DWORD)(csd[8] & 192) >> 6) |
	                            ((DWORD)csd[7] << 2) |
	                            ((DWORD)(csd[6] & 3) << 10);
	                    *(DWORD*)buff = (csize + 1) << (n - 9);
	                }
	                res = RES_OK;
	            }
	            CS_High(); SPI_RW(0xFF);
	            break;
	        }

	        case GET_SECTOR_SIZE:
	            *(WORD*)buff = 512;
	            res = RES_OK;
	            break;

	        case GET_BLOCK_SIZE:
	            *(DWORD*)buff = 128;
	            res = RES_OK;
	            break;

	        default:
	            res = RES_PARERR;
	    }

  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

