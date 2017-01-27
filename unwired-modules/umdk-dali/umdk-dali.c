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

#include "thread.h"
#include "xtimer.h"

static kernel_pid_t handler_pid;

static uwnds_cb_t *callback;

static xtimer_t pack_timer;
static msg_t pack_msg;

static uint32_t pack_dali;

void umdk_dali_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void)non_gpio_pin_map;

    callback = event_callback;

    gpio_init(UMDK_DALI_1, GPIO_IN_PU);

    /* Create handler thread */
     char *stack = (char *) allocate_stack();
     if (!stack) {
         puts("umdk-dali: unable to allocate memory. Is too many modules enabled?");
         return;
     }

     handler_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, handler, NULL, "Dali thread");

     xtimer_set_msg(&pack_timer, UMDK_DALI_TIME_TX_PACK_USEC, &pack_msg, handler_pid);

}


bool umdk_dali_cmd(module_data_t *cmd, module_data_t *reply)
{
    /* Check minimum command length */
    if (cmd->length < 2) {
	printf("[umdk-dali] Invalid command - wrong length of command\n");
        return false;
    }

    /* Check on the address type */
    /* if it isn't number of group or  isn't broadcast  */
    if((cmd->data[0] >> 7) == 1){
	if(( (cmd->data[0] >> 5) & 0x03) != 0) {
	    /* if it isn't broadcast  */
	    if((cmd->data[0] >> 1) != 0x7F) {
		printf("[umdk-dali] Invalid broadcast command\n");
		return false;
	    }
	    printf("[umdk-dali] Invalid type of address\n");
	    return false;
	}
    }

    /* Pack DALI is 19 bits */
    pack_dali = ((0 << 18) + (0 << 17) + (cmd->data[1] << 9) + (cmd->data[0] << 1) + (1 << 0)) & 0x0007FFFF;

    reply->length = 4;
    reply->data[0] = UNWDS_DALI_MODULE_ID;
    reply->data[1] = 'o';
    reply->data[2] = 'k';
    reply->data[3] = '\0';

    return true; /* Allow reply */

}


#ifdef __cplusplus
}
#endif
