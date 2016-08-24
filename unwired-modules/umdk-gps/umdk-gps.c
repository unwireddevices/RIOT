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
#include "umdk-gps.h"

#include "thread.h"
#include "xtimer.h"

static uwnds_cb_t *callback;

static bool has_data;
static sl3333_gps_data_t last_data;

static sl3333_t gps;

static void make_answer(char *buf) {
	/* This code composes an answer in JSON format */
    sprintf(buf, "{\"has_data\":true,\"lat\":\"%s\",\"lon\":\"%s\",\"n\":%s,\"e\":%s,\"date\":\"%s\",\"time\":\"%s\"}",
            last_data.lat, last_data.lon,
            (last_data.n) ? "true" : "false",
            (last_data.e) ? "true" : "false",
            last_data.date,
            last_data.time);
}

void gps_cb(sl3333_gps_data_t data)
{
    has_data = true;
    last_data = data;

#ifdef UMDK_GPS_AUTOPUBLISH
    if (callback != NULL) {
    	char buf[UNWDS_MAX_REPLY_LEN] = { '\0', };
    	make_answer(buf);
    	callback(buf);
    }
#endif
}

void umdk_gps_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    sl3333_param_t gps_params;
    gps_params.gps_cb = gps_cb;
    gps_params.uart = UMDK_GPS_UART;

    sl3333_init(&gps, &gps_params);
}

bool umdk_gps_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply)
{
    if (strcmp(argv[1], "get") == 0) {
        char buf[UNWDS_MAX_REPLY_LEN] = { '\0' };

        if (has_data) {
        	make_answer(buf);
        }
        else {
            sprintf(buf, "{has_data:false}");
        }

        strcpy(reply, buf);

        return true;
    }

    strcpy(reply, "{error: true, msg: \"invalid params\"}");

    return false;
}

#ifdef __cplusplus
}
#endif
