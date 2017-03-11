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
 * @file		umdk-lm75.c
 * @brief       umdk-lm75 module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"

#include "board.h"

#include "unwds-common.h"
#include "umdk-lm75.h"

#include "thread.h"
#include "xtimer.h"

#include "lm75a.h"

static uwnds_cb_t *callback;

static lm75a_t lm75a;

int umdk_lm75_shell_cmd(int argc, char **argv) {
    if (argc == 1) {
        puts ("lm75 get - get results now");
        return 0;
    }
    
    char *cmd = argv[1];
	
    if (strcmp(cmd, "get") == 0) {
        int temp = lm75a_get_ambient_temperature(&lm75a);
        
        char buf[10];
        int_to_float_str(buf, temp, 3);
        printf("[umdk-lm75] Temperature: %s C\n", buf);
    }
    
    return 1;
}

void umdk_lm75_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    lm75a.params.i2c = UMDK_LM75_I2C;
    lm75a.params.a1 = 0;
    lm75a.params.a2 = 0;
    lm75a.params.a3 = 0;

    if (lm75a_init(&lm75a)) {
        puts("[umdk-lm75] Error initializing LM75A sensor");
        return;
    }
    
    unwds_add_shell_command("lm75", "type 'lm75' for commands list", umdk_lm75_shell_cmd);
}

bool umdk_lm75_cmd(module_data_t *data, module_data_t *reply)
{
	if (data->length < 1)
		return false;

	umdk_lm75_cmd_t c = data->data[0];

	switch (c) {
	case UMDK_LM75_CMD_POLL:
	{
		int temp = lm75a_get_ambient_temperature(&lm75a);
		int16_t data = (temp/100);
		
		reply->length = 1 + sizeof(data);
		reply->data[0] = UNWDS_LM75_MODULE_ID;
		memcpy(reply->data + 1, &data, sizeof(data));
		break;
	}
	default:
		break;
	}

    return true;
}

#ifdef __cplusplus
}
#endif
