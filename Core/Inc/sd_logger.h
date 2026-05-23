/*
 * sd_logger.h
 *
 *  Created on: May 7, 2026
 *      Author: aboba
 */

#ifndef INC_SD_LOGGER_H_
#define INC_SD_LOGGER_H_

#include "fatfs.h"
#include <stdint.h>
#include "structs.h"

typedef enum {
    SD_LOG_OK    = 0,
    SD_LOG_ERROR = 1,
} SD_LogStatus;

SD_LogStatus SD_Logger_Init(void);
SD_LogStatus SD_Logger_Write(const NodeData *d);
void         SD_Logger_Flush(void);

#endif /* INC_SD_LOGGER_H_ */
