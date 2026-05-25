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

typedef enum {
    ESP_STATE_IDLE,
    ESP_STATE_SEND_CMD,
    ESP_STATE_WAIT_PROMPT,
    ESP_STATE_WAIT_OK
} ESP_State_t;

static ESP_State_t current_state = ESP_STATE_IDLE;
static char dma_cmd_buf[64];
static char dma_payload_buf[128];
static uint32_t state_timeout = 0;
static uint16_t rx_index = 0;
volatile uint8_t esp_rx_byte;

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

	// 1. Очищаємо глобальний буфер і скидаємо індекс
	memset(rxBuf, 0, sizeof(rxBuf));
	rx_index = 0;

	// 2. Про всяк випадок скидаємо апаратні помилки і гарантуємо,
	// що фонове переривання активно слухає ефір
	__HAL_UART_CLEAR_OREFLAG(&huart6);
	HAL_UART_Receive_IT(&huart6, (uint8_t*)&esp_rx_byte, 1);

	// 3. Пасивно чекаємо, поки переривання самостійно зловить потрібне слово
	while (HAL_GetTick() - start < timeout)
	{
		if (strstr((char*)rxBuf, expected_string) != NULL)
		{
			return 1; // Слово знайдено! Миттєвий вихід.
		}
		HAL_Delay(1); // Невеличка пауза, щоб не зависав IWDG watchdog
	}

	// Якщо таймаут вийшов, виводимо в лог те, що встигло прийти (для дебагу)
	if (strlen((char*)rxBuf) > 0) {
		debug_log("ESP_WAIT_DBG", (char*)rxBuf);
	}

	return 0;
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
    if (!ESP_WaitFor("OK", 500))
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
    if (!ESP_WaitFor("OK", 500))
    {
        debug_log("ESP01", "ATE0 failed");
    }

    // ======================
    // 4. MODE = STATION
    // ======================
    ESP_Send("AT+CWMODE=1");
    if (!ESP_WaitFor("OK", 1000))
    {
        debug_log("ESP01", "CWMODE failed");
    }

    // ======================
    // 5. CONNECT TO WIFI
    // ======================
    ESP_Send("AT+CWJAP=\"Big-Dick\",\"12344321\"");

    if (!ESP_WaitFor("OK", 8000))
    {
        debug_log("ESP01", "WiFi connect failed");
        return 0;
    }

    debug_log("ESP01", "INIT OK");
    HAL_UART_Receive_IT(&huart6, (uint8_t*)&esp_rx_byte, 1);
    return 1;
}

uint8_t ESP_ConnectTCP()
{
	ESP_Send("AT+CIPMUX=0");
	ESP_WaitFor("OK", 1000);

	ESP_Send("AT+CIPSTART=\"TCP\",\"192.168.0.198\",8888");

	// Чекаємо на універсальний OK замість CONNECT
	if (ESP_WaitFor("OK", 8000))
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

uint8_t ESP_SendMessage_Node_DMA(const NodeData *d)
{
    // Якщо ESP вже чимось зайнятий, ігноруємо відправку (щоб не зламати буфер)
    if (current_state != ESP_STATE_IDLE) {
        //debug_log("ESP01", "BUSY, packet skipped");
        return 0;
    }

    if (!esp_connected) {
        if (!ESP_ConnectTCP()) { // Це все ще блокуюча функція, але вона викликається рідко
            debug_log("ESP01", "NO TCP");
            return 0;
        }
    }

    // 1. Формуємо payload
    snprintf(dma_payload_buf, sizeof(dma_payload_buf), "%d;%d.%02d;%d.%02d;%d.%02d",
        d->id,
        (int)d->temp,     (int)((d->temp - (int)d->temp) * 100),
        (int)d->humidity, (int)((d->humidity - (int)d->humidity) * 100),
        (int)d->pressure, (int)((d->pressure - (int)d->pressure) * 100));

    // 2. Формуємо команду AT+CIPSEND
    snprintf(dma_cmd_buf, sizeof(dma_cmd_buf), "AT+CIPSEND=%d\r\n", (int)strlen(dma_payload_buf));

    // 3. Переводимо автомат у стан відправки
    current_state = ESP_STATE_SEND_CMD;
    return 1;
}

/* * ГОЛОВНИЙ ДВИГУН (Викликати в main while(1))
 * Ця функція виконується за мікросекунди і ніколи не блокує процесор.
 */
void ESP_Process(void)
{

    // 2. Логіка кінцевого автомата
    switch (current_state) {
        case ESP_STATE_IDLE:
            break; // Нічого не робимо

        case ESP_STATE_SEND_CMD:
			// 1. Жорстко перезапускаємо приймач, щоб скинути блокування HAL
			HAL_UART_AbortReceive(&huart6);
			__HAL_UART_CLEAR_OREFLAG(&huart6);

			// 2. Очищаємо програмний буфер
			memset(rxBuf, 0, sizeof(rxBuf));
			rx_index = 0;

			// 3. Гарантовано запускаємо переривання
			HAL_UART_Receive_IT(&huart6, (uint8_t*)&esp_rx_byte, 1);

			// 4. Відправляємо AT+CIPSEND через DMA
			HAL_UART_Transmit_DMA(&huart6, (uint8_t*)dma_cmd_buf, strlen(dma_cmd_buf));

			state_timeout = HAL_GetTick();
			current_state = ESP_STATE_WAIT_PROMPT;
			break;

        case ESP_STATE_WAIT_PROMPT:
			if (strstr((char*)rxBuf, ">") != NULL) {
				memset(rxBuf, 0, sizeof(rxBuf));
				rx_index = 0;

				HAL_UART_Transmit_DMA(&huart6, (uint8_t*)dma_payload_buf, strlen(dma_payload_buf));

				state_timeout = HAL_GetTick();
				current_state = ESP_STATE_WAIT_OK;
			}
			// ДОДАЄМО ЦЕЙ БЛОК: Швидке виявлення розірваного з'єднання
			else if (strstr((char*)rxBuf, "link is not") != NULL || strstr((char*)rxBuf, "ERROR") != NULL) {
				debug_log("ESP01", "TCP CONNECTION LOST!");
				esp_connected = 0; // Скидаємо прапорець
				current_state = ESP_STATE_IDLE; // Звільняємо автомат
			}
			else if ((HAL_GetTick() - state_timeout) > 3000) {
				debug_log("ESP01", "Timeout waiting for >");
				if (strlen((char*)rxBuf) > 0) debug_log("ESP_DBG", (char*)rxBuf);

				esp_connected = 0; // На всяк випадок теж скидаємо з'єднання
				current_state = ESP_STATE_IDLE;
			}
			break;

		case ESP_STATE_WAIT_OK:
			if (strstr((char*)rxBuf, "OK") != NULL || strstr((char*)rxBuf, "SEND OK") != NULL) {
				debug_log("ESP01", "DMA SEND SUCCESS");
				current_state = ESP_STATE_IDLE;
			}
			else if ((HAL_GetTick() - state_timeout) > 5000) {
				debug_log("ESP01", "Timeout waiting for OK");

				if (strlen((char*)rxBuf) > 0) {
					debug_log("ESP_DBG", (char*)rxBuf);
				} else {
					debug_log("ESP_DBG", "BUFFER IS EMPTY!");
				}

				esp_connected = 0;
				current_state = ESP_STATE_IDLE;
			}
			break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        if (rx_index < sizeof(rxBuf) - 1) {
            rxBuf[rx_index++] = esp_rx_byte;
            rxBuf[rx_index] = '\0'; // Закриваємо рядок для коректної роботи strstr
        }
        // Одразу запускаємо прослуховування наступного байта
        HAL_UART_Receive_IT(&huart6, (uint8_t*)&esp_rx_byte, 1);
    }
}


