/*
 * structs.h
 *
 *  Created on: May 12, 2026
 *      Author: aboba
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_
#include <stdint.h>

typedef struct {
    uint8_t id;
    float temp;
    float humidity;
    float pressure;
    uint8_t flags;
} NodeData;

NodeData parse_node_data(const char* str);

#endif /* INC_STRUCTS_H_ */
