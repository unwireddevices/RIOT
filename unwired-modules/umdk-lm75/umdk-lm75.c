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

void umdk_lm75_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    lm75a_param_t lm_params;
    lm_params.i2c = UMDK_LM75_I2C;

    lm75a_init(&lm75a, &lm_params);
}

bool umdk_lm75_cmd(module_data_t *data, module_data_t *reply)
{
	if (data->length < 1)
		return false;

	umdk_lm75_cmd_t c = data->data[0];

	switch (c) {
	case UMDK_LM75_CMD_POLL:
	{
		int16_t temp = lm75a_get_ambient_temperature(&lm75a);
		/* LM75A scale: signed, 0 = 0°C, 1 bit = 0.125 °C */
		/* our scale: signed, 1 bit = 0.1 °C */
		temp = (temp * 10) / 8;
		
		reply->length = 1 + sizeof(temp);
		reply->data[0] = UNWDS_LM75_MODULE_ID;
		memcpy(reply->data + 1, &temp, sizeof(temp));
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
