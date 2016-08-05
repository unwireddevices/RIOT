/*
 * Copyright (C) 2016 Unwired Devices
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
 * @file		unwds-common.c
 * @brief       Common routines for all UMDK modules
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "unwds-common.h"

#include "unwds-gpio.h"
#include "umdk-4btn.h"
#include "umdk-gps.h"
#include "umdk-temp.h"

/**
 * @brief Bitmap of occupied pins that cannot be used as gpio in-out
 */
static uint32_t non_gpio_pin_map;

/**
 * @brief Device abilities enabled by the used modules mask
 */
static uint64_t ability_map;

static const unwd_module_t modules[] = {
/*#ifdef UNWDS_GPIO
    { "gpio", unwds_gpio_init, unwds_gpio_cmd, 1 << 1 },
   #endif
   #ifdef UMDK_4BTN
    { "4btn", umdk_4btn_init, umdk_4btn_cmd, 1 << 2 },
 #endif*/
#ifdef UMDK_GPS
    { "gps", umdk_gps_init, umdk_gps_cmd, 1 << 3 },
#endif
#ifdef UMDK_TEMP
    { "temp", umdk_temp_init, umdk_temp_cmd, 1 << 4 },
#endif
    { "", NULL, NULL },
};

void unwds_init(uwnds_cb_t *event_callback)
{
    int i = 0;

    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
        printf("unwds: initializing \"%s\" module...\n", modules[i].name);
        modules[i].init_cb(&non_gpio_pin_map, event_callback);

        /* Set corresponding bits in ability map */
        ability_map |= modules[i].ability_mask;

        i++;
    }
}

static bool process_command(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply)
{
    char *module_name = argv[0];

    int i = 0;

    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
        if (strcmp(modules[i].name, module_name) == 0) {
            bool res = modules[i].cmd_cb(argc, argv, reply);
            char buf[UNWDS_MAX_REPLY_LEN] = { '\0' };
            sprintf(buf, "%s|%s", modules[i].name, reply);
            strcpy(reply, buf);

            return res;
        }

        i++;
    }

    strcpy(reply, "unknown command");

    return false;
}

bool unwds_command(char *command, char *reply)
{
    /* Tokenize the command parameters */
    char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN] = { { '\0', }, };
    int argc = 0, j = 0;
    int len = strlen(command);
    int i = 0;

    for (i = 0; i < len; i++) {
        if (argc >= UNWDS_MAX_PARAM_COUNT) {
            break;
        }

        if (command[i] == UNWDS_PARAM_DELIM) {
            argc++;
            j = 0;
        }
        else {
            if (j >= UNWDS_MAX_PARAM_LEN) {
                continue;
            }

            argv[argc][j++] = command[i];
        }
    }

    return process_command(argc, argv, reply);
}

bool unwds_is_pin_occupied(uint32_t pin)
{
    return ((non_gpio_pin_map >> pin) & 0x1);
}

uint64_t unwds_get_ability(void)
{
    return ability_map;
}

#ifdef __cplusplus
}
#endif
