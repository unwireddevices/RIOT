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
 * @file	umdk-4counter.c
 * @brief       umdk-4counter module implementation
 * @author      Mikhail Perkov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "lpm.h"

#include "board.h"

#include "unwds-common.h"
#include "umdk-4counter.h"

#include "thread.h"
#include "xtimer.h"
#include "rtctimers.h"

static kernel_pid_t handler_pid;

static uwnds_cb_t *callback;
static rtctimer_t counter_timer[4];
static rtctimer_t publishing_timer;

static uint8_t ignore_irq[4];

static msg_t counter_msg[4];
static msg_t publishing_msg = { .type = MESSAGE_TYPE_PUBLISHER };


static struct  {
    uint8_t is_valid;
    uint32_t count_value[UMDK_4COUNT_NUM_SENS];
    uint8_t publish_period;
} conf_counter;

static gpio_t pins_sens[UMDK_4COUNT_NUM_SENS] = { UMDK_4COUNT_1, UMDK_4COUNT_2, UMDK_4COUNT_3, UMDK_4COUNT_4 };

static void umdk_4count_counter_int(void* arg)
{
    int num = (int)arg;
    if (ignore_irq[num]) {
        return;
    }
    ignore_irq[num] = 1;
    
    gpio_irq_disable(pins_sens[num]);
    gpio_init(pins_sens[num], GPIO_AIN);
    
    uint8_t now_value = 0;
    /*
    uint8_t last_value = gpio_read(pins_sens[num]);
    uint8_t value_counter = 0;

    volatile int delay = 0;
    do {
        for (delay = 0; delay < 1000; delay++) {}
        now_value = gpio_read(pins_sens[num]);
        if (now_value == last_value) {
            value_counter++;
            last_value = now_value;
        } else {
            value_counter = 0;
        }
    }
    while (value_counter < 5);
    */
    
    /* increase counter value */
    if (now_value == 0) {
        conf_counter.count_value[num]++;
        /* Start counting timer */
        counter_msg[num].type = MESSAGE_TYPE_COUNTER;
        counter_msg[num].content.value = num;
        rtctimers_set_msg(&counter_timer[num], UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg[num], handler_pid);
    }
}

static void umdk_4count_counter_tim(uint32_t num)
{
    gpio_init(pins_sens[num], GPIO_IN_PU);
    __asm("nop; nop; nop; nop; nop;");
    uint8_t last_value = gpio_read(pins_sens[num]);
    __asm("nop; nop; nop; nop; nop;");
    gpio_init(pins_sens[num], GPIO_AIN);
    
    /* still zero, let's check again a bit later */
    if (last_value == 0) {
        /* Restart counting timer */
        counter_msg[num].type = MESSAGE_TYPE_COUNTER;
        counter_msg[num].content.value = num;
        rtctimers_set_msg(&counter_timer[num], UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg[num], handler_pid);
        return;
    }

    uint8_t now_value = 0;    
    uint8_t value_counter = 0;
    uint8_t error_counter = 0;
    volatile int delay = 0;
    
    do {
        for (delay = 0; delay < 32000; delay ++) {}
        
        gpio_init(pins_sens[num], GPIO_IN_PU);
        __asm("nop; nop; nop; nop; nop;");
        now_value = gpio_read(pins_sens[num]);
        __asm("nop; nop; nop; nop; nop;");
        gpio_init(pins_sens[num], GPIO_AIN);
        
        if (now_value == last_value) {
            value_counter++;
            last_value = now_value;
        } else {
            value_counter = 0;
            error_counter++;
        }
    } while ((value_counter < 5) && (error_counter < 100));
    
    /* shit is happening now, let's try later */
    if (error_counter > 100) {
        counter_msg[num].type = MESSAGE_TYPE_COUNTER;
        counter_msg[num].content.value = num;
        rtctimers_set_msg(&counter_timer[num], UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg[num], handler_pid);
        return;
    }
    
    /* if still 0, check a bit later */
    if (last_value == 0) {
        counter_msg[num].type = MESSAGE_TYPE_COUNTER;
        counter_msg[num].content.value = num;
        rtctimers_set_msg(&counter_timer[num], UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg[num], handler_pid);
        return;
    }
    
    /* enable pull-up, wait for next interrupt */
    gpio_init(pins_sens[num], GPIO_IN_PU);
    ignore_irq[num] = 0;
    gpio_irq_enable(pins_sens[num]);
}

static inline void save_config(void)
{
    conf_counter.is_valid = 1;
    unwds_write_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter));
}

static void *handler(void *arg)
{
    msg_t msg;
    msg_t msg_queue[16];
    msg_init_queue(msg_queue, 16);

    while (1) {
        msg_receive(&msg);
        umdk_4counter_msg_t type = msg.type;

        switch (type) {
            case COUNTING:
                umdk_4count_counter_tim(msg.content.value);
                printf("Counting, channel %lu\n", msg.content.value);
                break;

            case PUBLISHING:
            	puts("Sending");

                module_data_t data;
                data.length = 1 + 4 * UMDK_4COUNT_NUM_SENS;

                /* Write module ID */
                data.data[0] = UNWDS_4COUNTER_MODULE_ID;

                /* Write four counter values */
                uint32_t *tmp = (uint32_t *)(&data.data[1]);

                /* Compress 4 values to 12 bytes total */
                *(tmp + 0)  = conf_counter.count_value[0] << 8;
                *(tmp + 0) |= (conf_counter.count_value[1] >> 16) & 0xFF;
                
                *(tmp + 1) = conf_counter.count_value[1] << 16;
                *(tmp + 1) |= (conf_counter.count_value[2] >> 8) & 0xFFFF;
                
                *(tmp + 2) = (conf_counter.count_value[2] << 24);
                *(tmp + 2) |= conf_counter.count_value[3] & 0xFFFFFF;

                save_config(); /* Save values into NVRAM */

                callback(&data);

                /* Restart timer */
                if (conf_counter.publish_period) {
                    rtctimers_set_msg(&publishing_timer, \
                                      UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, \
                                      &publishing_msg, handler_pid);
                }
                gpio_irq_enable(UMDK_4COUNT_BTN);
                break;

            default:
                break;
        }
    }
    return NULL;
}

static void umdk_4count_connect(void* arg) {
    /* connect button pressed — publish to LoRa in 1 second */
    gpio_irq_disable(UMDK_4COUNT_BTN);
    rtctimers_set_msg(&publishing_timer, 1, &publishing_msg, handler_pid);
}

static void reset_config(void) {
	conf_counter.is_valid = 0;
	memset(&conf_counter.count_value[0], 0, sizeof(conf_counter.count_value));
	conf_counter.publish_period = UMDK_4COUNT_PUBLISH_PERIOD_MIN;
}

void umdk_4counter_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    conf_counter.publish_period = UMDK_4COUNT_PUBLISH_PERIOD_MIN;

    callback = event_callback;

    for (int i = 0; i < UMDK_4COUNT_NUM_SENS; i++) {
        gpio_init_int(pins_sens[i], GPIO_IN_PU, GPIO_FALLING, umdk_4count_counter_int, (void *) i);
        ignore_irq[i] = 0;
    }
    
    gpio_init_int(UMDK_4COUNT_BTN, GPIO_IN_PU, GPIO_FALLING, umdk_4count_connect, NULL);

    /* Create handler thread */
    char *stack = (char *) allocate_stack();
    if (!stack) {
        puts("umdk-4counter: unable to allocate memory. Is too many modules enabled?");
        return;
    }

    /* Load config from NVRAM */
    if (!unwds_read_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter))) {
        return;
    }
    
    if ((conf_counter.is_valid == 0xFF) || (conf_counter.is_valid == 0))  {
		reset_config();
	}

    printf("[umdk-4counter] Current publish period: %d hour(s)\n", conf_counter.publish_period);

    handler_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, \
                                THREAD_CREATE_STACKTEST, handler, NULL, "4counter thread");

    /* Start publishing timer */
    rtctimers_set_msg(&publishing_timer, \
                      UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, \
                      &publishing_msg, handler_pid);

    /* Start counting timer */
    /* rtctimers_set_msg(&counter_timer, UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg, handler_pid); */
}


bool umdk_4counter_cmd(module_data_t *cmd, module_data_t *reply)
{
    if (cmd->length < 1) {
        return false;
    }

    umdk_4counter_cmd_t c = cmd->data[0];
    switch (c) {
        case UMDK_4COUNT_CMD_SET_PERIOD: {
            if (cmd->length != 2) {
                return false;
            }

            uint8_t period = cmd->data[1];
            /* do not change period if new one is 0 or > max */
            if ((!period) || (period > UMDK_4COUNT_PUBLISH_PERIOD_MAX)) {
                reply->length = 2;
                reply->data[0] = UNWDS_4COUNTER_MODULE_ID;
                reply->data[1] = 253;
                return true;
            }
            
            rtctimers_remove(&publishing_timer);

            conf_counter.publish_period = period;
            save_config();

            rtctimers_set_msg(&publishing_timer, \
                              UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, \
                              &publishing_msg, handler_pid);
            printf("[umdk-4counter] Period set to %d hour (s)\n", conf_counter.publish_period);

            reply->length = 4;
            reply->data[0] = UNWDS_4COUNTER_MODULE_ID;
            reply->data[1] = 'o';
            reply->data[2] = 'k';
            reply->data[3] = '\0';

            return true; /* Allow reply */
        }

        case UMDK_4COUNTER_CMD_POLL:

            /* Send values to publisher thread */
            msg_send(&publishing_msg, handler_pid);

            return false; /* Don't reply */

            break;

        default:
            break;
    }

    /* Don't reply by default */
    return false;
}

#ifdef __cplusplus
}
#endif
