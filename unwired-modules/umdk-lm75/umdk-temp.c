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
 * @file		umdk-4btn.c
 * @brief       umdk-4btn module implementation
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

bool umdk_lm75_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply)
{
    if (strcmp(argv[1], "get") == 0) {
        char buf[UNWDS_MAX_REPLY_LEN] = { '\0' };

        float_t temp = lm75a_get_ambient_temperature(&lm75a);
        sprintf(buf, "{temp:%.3f}", temp);
        strcpy(reply, buf);

        return true;
    }

    strcpy(reply, "{error:true, text:\"invalid params\"}");

    return false;
}

#ifdef __cplusplus
}
#endif
