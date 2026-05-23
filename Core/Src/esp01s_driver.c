/*
 * esp01s_driver.c
 *
 *  Created on: Feb 6, 2026
 *      Author: aboba
 */
#include "esp01s_driver.h"

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "debug.h"

extern UART_HandleTypeDef huart6;
extern UART_HandleTypeDef huart1;
static uint8_t rxBuf[512];
static uint8_t esp_connected = 0;

void ESP_Send(char *cmd)
{
    char buf[256];
    sprintf(buf, "%s\r\n", cmd);
    HAL_UART_Transmit(&huart6, (uint8_t*)buf, strlen(buf), 1000);
}

void ESP_Receive(uint8_t *buffer, uint16_t size)
{
    memset(buffer, 0, size);
    HAL_UART_Receive(&huart6, buffer, size, 3000);
}

void ESP_ReceiveTimeout(uint8_t *buffer, uint16_t size, uint32_t timeout)
{
    memset(buffer, 0, size);
    HAL_UART_Receive(&huart6, buffer, size, timeout);
}

uint8_t ESP_WaitFor(const char* expected_string, uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    uint16_t index = 0;

    memset(rxBuf, 0, sizeof(rxBuf));

    while (HAL_GetTick() - start < timeout)
    {
        uint8_t ch;
        // Читаємо по 1 байту з маленьким таймаутом (наприклад, 10 мс)
        if (HAL_UART_Receive(&huart6, &ch, 1, 10) == HAL_OK)
        {
            rxBuf[index++] = ch;

            // Захист від переповнення буфера
            if (index >= sizeof(rxBuf) - 1) {
                index = 0;
                memset(rxBuf, 0, sizeof(rxBuf));
            }

            rxBuf[index] = '\0'; // Завжди тримаємо рядок закритим

            // Якщо знайшли те, що шукали — миттєво виходимо!
            if (strstr((char*)rxBuf, expected_string) != NULL)
            {
                return 1;
            }
        }
    }
    return 0; // Таймаут вийшов, а слова не знайшли
}

uint8_t ESP_WaitOK(uint32_t timeout)
{
    ESP_ReceiveTimeout(rxBuf, sizeof(rxBuf), timeout);
    return (strstr((char*)rxBuf, "OK") != NULL);
}

uint8_t ESP_Init()
{

    // ======================
    // 1. BASIC AT CHECK
    // ======================
    ESP_Send("AT");
    if (!ESP_WaitOK(500))
    {
        debug_log("ESP01", "AT no response");
        return 0;
    }

    // ======================
    // 2. RESET (не завжди OK приходить стабільно)
    // ======================
    ESP_Send("AT+RST");
    ESP_ReceiveTimeout(rxBuf, sizeof(rxBuf), 3000);
    HAL_Delay(2000); // ESP реально перезавантажується

    // ======================
    // 3. ECHO OFF
    // ======================
    ESP_Send("ATE0");
    if (!ESP_WaitOK(500))
    {
        debug_log("ESP01", "ATE0 failed");
    }

    // ======================
    // 4. MODE = STATION
    // ======================
    ESP_Send("AT+CWMODE=1");
    if (!ESP_WaitOK(1000))
    {
        debug_log("ESP01", "CWMODE failed");
    }

    // ======================
    // 5. CONNECT TO WIFI
    // ======================
    ESP_Send("AT+CWJAP=\"Big-Dick\",\"12344321\"");

    if (!ESP_WaitOK(8000))
    {
        debug_log("ESP01", "WiFi connect failed");
        return 0;
    }

    debug_log("ESP01", "INIT OK");
    return 1;
}

uint8_t ESP_ConnectTCP()
{
    ESP_Send("AT+CIPMUX=0");
    ESP_ReceiveTimeout(rxBuf, sizeof(rxBuf), 1000);

    ESP_Send("AT+CIPSTART=\"TCP\",\"192.168.0.198\",8888");
    ESP_ReceiveTimeout(rxBuf, sizeof(rxBuf), 8000);

    if (strstr((char*)rxBuf, "OK") || strstr((char*)rxBuf, "CONNECT"))
    {
    	debug_log("ESP01","CONNECT SUCCESS");
        esp_connected = 1;
        return 1;
    }

    debug_log("ESP01","CONNECT UNSUCCESS");
    esp_connected = 0;
    return 0;
}

void ESP_SendMessage(char* msg)
{
    if (!esp_connected)
    {
        if (!ESP_ConnectTCP())
        {
            debug_log("ESP01", "NO TCP");
            return;
        }
    }

    char cmd[64];
    sprintf(cmd, "AT+CIPSEND=%d", (int)strlen(msg));
    ESP_Send(cmd);

    if (!ESP_WaitFor(">", 3000))
        {
            debug_log("ESP01", "NO > prompt");
            esp_connected = 0;
            return;
        }

    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), 1000);

    if (!ESP_WaitFor("OK", 5000)) {
            debug_log("ESP01", "SEND FAIL (Timeout)");
            esp_connected = 0;
        }
}

void ESP_SendMessage_Node(const NodeData *d)
{
    char buf[64];
    snprintf(buf, sizeof(buf), "%d;%d.%02d;%d.%02d;%d.%02d",
        d->id,
        (int)d->temp,     (int)((d->temp - (int)d->temp) * 100),
        (int)d->humidity, (int)((d->humidity - (int)d->humidity) * 100),
        (int)d->pressure, (int)((d->pressure - (int)d->pressure) * 100));
    ESP_SendMessage(buf);
}


