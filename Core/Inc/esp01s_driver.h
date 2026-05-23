/*
 * esp01s_driver.h
 *
 *  Created on: Feb 6, 2026
 *      Author: aboba
 */

#ifndef INC_ESP01S_DRIVER_H_
#define INC_ESP01S_DRIVER_H_

#include <stdint.h>
#include "structs.h"


// Init WiFi + ESP
uint8_t ESP_Init(void);

// TCP connection management
uint8_t ESP_ConnectTCP(void);

// Send data over TCP
void ESP_SendMessage(char *msg);
void ESP_SendMessage_Node(const NodeData *d);

// Low-level ESP AT interface
void ESP_Send(char *cmd);
void ESP_Receive(uint8_t *buffer, uint16_t size);
void ESP_ReceiveTimeout(uint8_t *buffer, uint16_t size, uint32_t timeout);

// Utility
uint8_t ESP_WaitOK(uint32_t timeout);
#endif /* INC_ESP01S_DRIVER_H_ */
