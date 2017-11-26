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
#include "board.h"
#include "unwds-common.h"
#include "periph/gpio.h"
#include "rtctimers-millis.h"

void blink_led(gpio_t led)
{
    int i;
    gpio_set(led);
    for (i = 0; i < 4; i++) {
        gpio_toggle(led);
        rtctimers_millis_sleep(50);        
    }
    gpio_clear(led);
}

void print_logo(void)
{
	puts("*****************************************");
	puts("Unwired Range firmware by Unwired Devices");
	puts("www.unwds.com - info@unwds.com");
#ifdef NO_RIOT_BANNER
    puts("powered by RIOT - www.riot-os.org");
#endif
	puts("*****************************************");
    printf("Version: %s (%s %s)\n", FIRMWARE_VERSION, __DATE__, __TIME__);
    char cpu_model[20];
    switch (get_cpu_category()) {
        case 1:
            snprintf(cpu_model, 20, "STM32L151CB");
            break;
        case 2:
            snprintf(cpu_model, 20, "STM32L151CB-A");
            break;
        case 3:
            snprintf(cpu_model, 20, "STM32L151CC");
            break;
    }
    printf("%s %lu MHz (%s clock)\n", cpu_model,
                                      cpu_clock_global/1000000,
                                      cpu_clock_source);
    printf("%lu KB RAM, %lu KB flash, %lu KB EEPROM\n\n", get_cpu_ram_size()/1024,
                                                          get_cpu_flash_size()/1024,
                                                          get_cpu_eeprom_size()/1024);
}

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
	int i;
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
