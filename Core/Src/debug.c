/*
 * debug.c
 *
 *  Created on: May 5, 2026
 *      Author: aboba
 */

#include "debug.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

void debug_log(const char *module, const char *msg)
{
    char buf[128];
    snprintf(buf, sizeof(buf), "[%s] %s\r\n", module, msg);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
}
/*
void debug_logf(const char *module, const char *fmt, ...)
{
    char msg[128];

    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    debug_log(module, msg);
}*/
