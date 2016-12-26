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

#include "board.h"

#include "unwds-common.h"
#include "umdk-4counter.h"

#include "thread.h"
#include "xtimer.h"
#include "rtc-timers.h"


static uint8_t time_detect = (uint8_t)(UMDK_4COUNT_DEBOUNCE_TIME_MS / UMDK_4COUNT_DETECT_COUNT);

static kernel_pid_t handler_pid;

static uwnds_cb_t *callback;
static rtctimer_t counter_timer;
static rtctimer_t publishing_timer;

static msg_t counter_msg = { .type = 0 };
static msg_t publishing_msg = { .type = 1 };


static struct  {
    uint8_t is_valid;
    uint32_t count_value[UMDK_4COUNT_NUM_SENS];
    uint8_t publish_period;
} conf_counter;

static gpio_t pins_sens[UMDK_4COUNT_NUM_SENS] = { UMDK_4COUNT_1, UMDK_4COUNT_2, UMDK_4COUNT_3, UMDK_4COUNT_4 };


static void umdk_4count_gpio_mode(gpio_t pin, gpio_mode_t mode, umdk_4counter_signal_t signal  )
{
    GPIO_TypeDef *port = (GPIO_TypeDef *)(pin & ~(0x0f));

    int pin_num =  (pin & 0x0f);

    umdk_4counter_signal_t sign = signal;

    switch (sign) {
        case DIGITAL:
            /* set mode */
            port->MODER &= ~(0x3 << (2 * pin_num));
            port->MODER |=  ((mode & 0x3) << (2 * pin_num));

            /* set pull resistor configuration */
            port->PUPDR &= ~(0x3 << (2 * pin_num));
            port->PUPDR |=  (((mode >> 2) & 0x3) << (2 * pin_num));
            break;

        case ANALOG:
            port->MODER &= ~(0x3 << (2 * pin_num));
            port->MODER |= (0x3 << (2 * pin_num));
            break;

        default:
            break;
    }
}

static void umdk_4count_counter_int(void)
{

    int8_t now_value[UMDK_4COUNT_NUM_SENS] = { 0 };
    int8_t last_value[UMDK_4COUNT_NUM_SENS] = { 0 };
    int8_t accept_value[UMDK_4COUNT_NUM_SENS] = { 1 };

    /* Set GPIO mode: Digital with pull-up */
    for (int i = 0; i < UMDK_4COUNT_NUM_SENS; i++) {
        umdk_4count_gpio_mode(pins_sens[i], GPIO_IN_PU, DIGITAL);
    }

    /* Detecting impulses */
    for (int8_t i = 0; i < UMDK_4COUNT_DETECT_COUNT; i++) {
        for (int j = 0; j < UMDK_4COUNT_NUM_SENS; j++) {
            /* Read the value from gpio pins */
            now_value[j] = !gpio_read(pins_sens[j]);

            if (now_value[j] == 0) {
                accept_value[j] = 0;
            }
            else if (last_value[j] == 1) {
                accept_value[j] = i + 1;
            }

            last_value[j] = now_value[j];

            /* Increase pulses count for current input */
            if (accept_value[j] == UMDK_4COUNT_DETECT_COUNT) {
                conf_counter.count_value[j]++;
            }
        }
        /* Delay */
        xtimer_usleep(time_detect * 1000);
    }

    /* Set GPIO mode: Analog */
    for (int i = 0; i < UMDK_4COUNT_NUM_SENS; i++) {
        umdk_4count_gpio_mode(pins_sens[i], GPIO_IN_PU, ANALOG);
    }

    /* Restart counting timer */
    rtctimers_set_msg(&counter_timer, UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg, handler_pid);
}

static inline void save_config(void)
{
    conf_counter.is_valid = 1;
    unwds_write_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter));
}

static void *handler(void *arg)
{
    msg_t msg;
    msg_t msg_queue[4];

    msg_init_queue(msg_queue, 4);

    while (1) {
        msg_receive(&msg);

        umdk_4counter_msg_t type = msg.type;

        switch (type) {
            case COUNTING:
                umdk_4count_counter_int();
                break;

            case PUBLISHING:
            	;

                module_data_t data;
                data.length = 1 + 4 * UMDK_4COUNT_NUM_SENS;

                /* Write module ID */
                data.data[0] = UNWDS_4COUNTER_MODULE_ID;

                /* Write four counter values */
                uint32_t *tmp = (uint32_t *)(&data.data[1]);

                *(tmp + 0)  = conf_counter.count_value[0];
                *(tmp + 1)  = conf_counter.count_value[1];
                *(tmp + 2)  = conf_counter.count_value[2];
                *(tmp + 3)  = conf_counter.count_value[3];

                save_config(); /* Save values into NVRAM */

                callback(&data);

                /* Restart timer */
                if (conf_counter.publish_period) {
                    rtctimers_set_msg(&publishing_timer, UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, &publishing_msg, handler_pid);
                }
                break;

            default:
                break;
        }
    }

    return NULL;
}



void umdk_4counter_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    conf_counter.publish_period = UMDK_4COUNT_PUBLISH_PERIOD_MIN;

    callback = event_callback;

    for (int i = 0; i < UMDK_4COUNT_NUM_SENS; i++) {
        gpio_init(pins_sens[i], GPIO_IN_PU);
    }

    /* Create handler thread */
    char *stack = (char *) allocate_stack();
    if (!stack) {
        puts("umdk-4counter: unable to allocate memory. Is too many modules enabled?");
        return;
    }

    /* Load config from NVRAM */
    unwds_read_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter));

    printf("[umdk-4counter] Current publish period: %d hour(s)\n", conf_counter.publish_period);

    handler_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, handler, NULL, "4counter thread");

    /* Start publishing timer */
    rtctimers_set_msg(&publishing_timer, UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, &publishing_msg, handler_pid);

    /* Start counting timer */
    rtctimers_set_msg(&counter_timer, UMDK_4COUNT_SLEEP_TIME_SEC, &counter_msg, handler_pid);
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
            rtctimers_remove(&publishing_timer);

            conf_counter.publish_period = period;
            save_config();

            /* Don't restart timer if new period is zero and if new period more max */
            if ((conf_counter.publish_period) && (conf_counter.publish_period < (UMDK_4COUNT_PUBLISH_PERIOD_MAX + 1))) {
                rtctimers_set_msg(&publishing_timer, UMDK_4COUNT_VALUE_PERIOD_PER_SEC * conf_counter.publish_period, &publishing_msg, handler_pid);
                printf("[umdk-4counter] Period set to %d hour (s)\n", conf_counter.publish_period);
            }
            else {
                puts("[umdk-4counter] Timer stopped");
            }

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
