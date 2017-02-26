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
 * @file	umdk-rssiecho.c
 * @brief       umdk-rssiecho module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-rssiecho.h"

#include "thread.h"

static uwnds_cb_t *callback;

void umdk_rssiecho_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;
    callback = event_callback;
}

bool umdk_rssiecho_cmd(module_data_t *cmd, module_data_t *reply)
{
    if (cmd->length < 1) {
        return false;
    }

    umdk_rssiecho_cmd_t c = cmd->data[0];
    switch (c) {
        case UMDK_RSSIECHO_CMD_ECHO: {
            /* Copy RSSI value into reply */
            int16_t rssi = cmd->rssi;
            reply->length = 1 + sizeof(rssi);
            reply->data[0] = UNWDS_RSSIECHO_MODULE_ID;
            memcpy(&reply->data[1], (uint8_t *) &rssi, sizeof(rssi));

            return true;
        }

        default:
            break;
    }

    return false;
}

#ifdef __cplusplus
}
#endif
