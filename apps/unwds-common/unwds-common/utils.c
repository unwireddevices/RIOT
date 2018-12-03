/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

#include "fmt.h"
#include "utils.h"
#include "board.h"
#include "unwds-common.h"
#include "periph/gpio.h"
#include "rtctimers-millis.h"

#include "rtctimers.h"

bool hex_to_bytes(char *hexstr, uint8_t *bytes, bool reverse_order) {
    uint32_t len = strlen(hexstr);
    while(true) {
        if (hexstr[len-1] == '\r' || hexstr[len-1] == '\n') {
            len--;
        } else {
            break;
        }
    }
    
	return hex_to_bytesn(hexstr, len, bytes, reverse_order);
}

static uint8_t ascii_to_number(char ascii) {
    if (ascii < 'A') {
        return (ascii - '0');
    } else if (ascii < 'a') {
        return (ascii - 'A' + 10);
    } else {
        return (ascii - 'a' + 10);
    }
}

static uint8_t hex_to_number(char *hex) {
    return ((ascii_to_number(*hex) << 4) | ascii_to_number(*(hex + 1)));
}

bool hex_to_bytesn(char *hexstr, int len, uint8_t *bytes, bool reverse_order) {
	/* Length must be even */
	if (len % 2 != 0)
		return false;

	/* Move in string by two characters */
	char *ptr = &(*hexstr);
	int i = 0;
	if (reverse_order) {
		ptr += len - 2;

		for (; (len >> 1) - i; ptr -= 2) {
			uint8_t v = 0;
            v = hex_to_number(ptr);
			bytes[i++] = v;
		}
	} else {
		for (; *ptr; ptr += 2) {
			uint8_t v = 0;
			v = hex_to_number(ptr);

			bytes[i++] = v;
		}
	}

	return true;
}

void bytes_to_hex(uint8_t *bytes, size_t num_bytes, char *str, bool reverse_order) {
    if (reverse_order) {
        fmt_bytes_hex_reverse(str, bytes, num_bytes);
    } else {
        fmt_bytes_hex(str, bytes, num_bytes);
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
