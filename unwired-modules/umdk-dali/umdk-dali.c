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
 * @file	umdk-dali.c
 * @brief       umdk-dali module implementation
 * @author      Mikhail Perkov

 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "board.h"
#include "periph/gpio.h"

#include "unwds-common.h"
#include "include/umdk-dali.h"


static uwnds_cb_t *callback;

void umdk_dali_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void)non_gpio_pin_map;

    callback = event_callback;


}


bool umdk_dali_cmd(module_data_t *cmd, module_data_t *reply)
{
    /* Check minimum command length */
    if (cmd->length < 6) {
	printf("[umdk-dali] Invalid command - wrong length of command\n");
        return false;
    }

    return false;
}


#ifdef __cplusplus
}
#endif
