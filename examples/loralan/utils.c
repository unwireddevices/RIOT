/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file
 * @brief       
 * @author      Evgeniy Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "utils.h"

bool hex_to_bytes(char *hexstr, uint8_t *bytes, bool reverse_order) {
	return hex_to_bytesn(hexstr, strlen(hexstr), bytes, reverse_order);
}

bool hex_to_bytesn(char *hexstr, uint32_t len, uint8_t *bytes, bool reverse_order) {
	/* Length must be even */
	if (len % 2 != 0)
		return false;

	/* Move in string by two characters */
	char *ptr = &(*hexstr);
	uint32_t i = 0;
	if (reverse_order) {
		ptr += len - 2;

		for (; (len >> 1) - i; ptr -= 2) {
			unsigned int v = 0;
			sscanf(ptr, "%02x", &v);

			bytes[i++] = (uint8_t) v;
		}
	} else {
		for (; *ptr; ptr += 2) {
			unsigned int v = 0;
			sscanf(ptr, "%02x", &v);

			bytes[i++] = (uint8_t) v;
		}
	}

	return true;
}

void bytes_to_hex(uint8_t *bytes, size_t num_bytes, char *str, bool reverse_order) {
	uint32_t i;
	for (i = 0; i < num_bytes; i++) {
		char buf[2];
		sprintf(buf, "%02x", bytes[(reverse_order) ? num_bytes - 1 - i : i]);
		strcat(str, buf);
	}
}

bool is_number(char* str) {
    char *endptr = NULL;
    strtol(str, &endptr, 0);
    
    if ( &str[strlen(str)] == endptr  ) {
        return true;
    } else {
        return false;
    }
}

#ifdef __cplusplus
}
#endif
