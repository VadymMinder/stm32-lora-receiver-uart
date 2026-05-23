/*
 * structs.c
 *
 *  Created on: May 12, 2026
 *      Author: aboba
 */


#include "structs.h"
#include <string.h>
#include <stdlib.h>

NodeData parse_node_data(const char* str) {
    NodeData d = {0};
    char buf[64];
    strncpy(buf, str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *token = strtok(buf, ";");
    if (token) d.id       = atoi(token);
    token = strtok(NULL, ";");
    if (token) d.temp     = atof(token);
    token = strtok(NULL, ";");
    if (token) d.humidity = atof(token);
    token = strtok(NULL, ";");
    if (token) d.pressure = atof(token);

    return d;
}
