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

static uwnds_cb_t *callback;

static bool has_data;
static sl3333_gps_data_t last_data;

static sl3333_t gps;

static void make_reply(module_data_t *reply) {
    printf("[gps] {\"has_data\":true,\"lat\":\"%s\",\"lon\":\"%s\",\"n\":%s,\"e\":%s,\"date\":\"%s\",\"time\":\"%s\"}\n",
            last_data.lat, last_data.lon,
            (last_data.n) ? "true" : "false",
            (last_data.e) ? "true" : "false",
            last_data.date,
            last_data.time);

    float float_lat, float_lon;
    float_lat = atof(last_data.lat);
    float_lon = atof(last_data.lon);

    /* Negative latitude and longitude */
    uint8_t lat_neg = (last_data.n) ? 0 : 1;
    uint8_t lon_neg = (last_data.e) ? 0 : 1;

    uint8_t coords[6] = {}; /* 6*8 bits = 48 */
    int32_t lat = float_lat * 1000;
    int32_t lon = float_lon * 1000;

    /* Pad 2 int32_t to 6 8uint_t, skipping the last byte (x >> 24) */
    coords[0] = lat;
    coords[1] = lat >> 8;
    coords[2] = lat >> 16;

    coords[3] = lon;
    coords[4] = lon >> 8;
    coords[5] = lon >> 16;

    reply->data[0] = UNWDS_GPS_MODULE_ID;
    reply->data[1] = UMDK_GPS_REPLY_GPS_DATA | (lat_neg << 5) | (lon_neg << 6); /* Bits 5 and 6 is markers of lat and lon sign */
    memcpy(reply->data + 2, coords, sizeof(coords)); /* Copy 48 bit GPS data */

    reply->length = 1 + 1 + sizeof(coords); /* [module ID] + [reply byte] + [48 bit GPS data] */
}

void gps_cb(sl3333_gps_data_t data)
{
    has_data = true;
    last_data = data;
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

bool umdk_gps_cmd(module_data_t *data, module_data_t *reply)
{
	umdk_gps_cmd_t cmd = data->data[0];

	reply->data[0] = UNWDS_GPS_MODULE_ID;

	if (data->length > 0) {
		switch (cmd) {
		case UMDK_GPS_CMD_POLL:
			if (has_data) {
				make_reply(reply);
			} else {
				reply->data[1] = UMDK_GPS_REPLY_NO_DATA;
				reply->length = 2; /* Module ID + reply byte */
			}

			return true;

		default:
			break;
		}
	}

	reply->data[1] = UMDK_GPS_REPLY_ERROR; /* Unknown command */
    reply->length = 2;

    return true; /* Send reply */
}

#ifdef __cplusplus
}
#endif
