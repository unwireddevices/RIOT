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
 * @file		umdk-lmt01.c
 * @brief       umdk-lmt01 module implementation
 * @author      Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "periph/gpio.h"
#include "periph/adc.h"
#include "board.h"

#include "unwds-common.h"

#include "umdk-6adc.h"

#include "thread.h"
#include "rtctimers.h"

static uwnds_cb_t *callback;

static kernel_pid_t timer_pid;

static msg_t timer_msg = {};
static rtctimer_t timer;

static bool is_polled = false;

static struct {
	uint8_t is_valid;
	uint8_t publish_period_min;

	bool adc_lines_en[ADC_NUMOF];

	gpio_t out_pin;
} adc_config;

static void reset_config(void) {
	adc_config.is_valid = 0;

	adc_config.publish_period_min = UMDK_6ADC_PUBLISH_PERIOD_MIN;

	for (int i = 0; i < ADC_NUMOF; i++) {
		adc_config.adc_lines_en[i] = true;
	}

	adc_config.out_pin = UMDK_6ADC_OUT_PIN;
}

static void init_config(void) {
	reset_config();

	if (!unwds_read_nvram_config(UNWDS_6ADC_MODULE_ID, (uint8_t *) &adc_config, sizeof(adc_config)))
		return;

	if ((adc_config.is_valid == 0xFF) || (adc_config.is_valid == 0)) {
		reset_config();
		return;
	}

	printf("[6adc] Publish period: %d min\n", adc_config.publish_period_min);
}

static inline void save_config(void) {
	adc_config.is_valid = 1;
	unwds_write_nvram_config(UNWDS_6ADC_MODULE_ID, (uint8_t *) &adc_config, sizeof(adc_config));
}

static void init_gpio(void)
{
    gpio_init(adc_config.out_pin, GPIO_OUT);
    gpio_clear(adc_config.out_pin);
}

static void init_adc(bool enable_all)
{
    int i = 0;

    for (i = 0; i < ADC_NUMOF; i++) {
        if (adc_init(ADC_LINE(i)) < 0) {
            printf("[umdk-6adc] Failed to initialize adc line #%d\n", i + 1);
            continue;
        }

        if (enable_all) {
            adc_config.adc_lines_en[i] = true;
        }
    }
}

static void prepare_result(module_data_t *buf)
{
    int i;

    uint16_t samples[ADC_NUMOF] = {};

    for (i = 0; i < ADC_NUMOF; i++) {
        if (!adc_config.adc_lines_en[i]) {
            samples[i] = 0xFFFF;
            printf("[umdk-6adc] Reading line #%d: <disabled>\n", i + 1);
            continue;
        }

        samples[i] = adc_sample(ADC_LINE(i), UMDK_6ADC_ADC_RESOLUTION);
    }
	
	if (UMDK_6ADC_CONVERT_TO_MILLIVOLTS) {
		/* Calculate Vdd */
		uint32_t full_scale = 0;
		
		switch (UMDK_6ADC_ADC_RESOLUTION) {
			case ADC_RES_12BIT:
				full_scale = 4095;
				break;
			case ADC_RES_10BIT:
				full_scale = 1023;
				break;
			case ADC_RES_8BIT:
				full_scale = 255;
				break;
			case ADC_RES_6BIT:
				full_scale = 63;
				break;
			default:
				puts("[umdk-6adc] Unsupported ADC resolution, aborting.");
				return;
				break; 
		}
		
		for (i = 0; i < ADC_NUMOF; i++) {
			if ((i != ADC_VREF_INDEX) && (i != ADC_TEMPERATURE_INDEX)) {
				samples[i] = (uint32_t)(samples[i] * samples[ADC_VREF_INDEX]) / full_scale;
			}
		}
	}
	
	for (i = 0; i < ADC_NUMOF; i++) {
		printf("[umdk-6adc] Reading line #%d: %d", i + 1, samples[i]);
		if (UMDK_6ADC_CONVERT_TO_MILLIVOLTS) {
			if (i == ADC_TEMPERATURE_INDEX) {
				puts(" C");
			}
			else {
				puts(" mV");
			}
		}
		else {
			puts(" ");
		}
	}

    buf->data[0] = UNWDS_6ADC_MODULE_ID;
    memcpy(buf->data + 1, (uint8_t *) &samples, sizeof(samples));
    buf->length = sizeof(samples) + 1; /* Additional byte for module ID */
}

static void *timer_thread(void *arg)
{
    msg_t msg;
    msg_t msg_queue[4];
    msg_init_queue(msg_queue, 4);

    puts("[umdk-6adc] Periodic publisher thread started");

    while (1) {
        msg_receive(&msg);

        gpio_set(adc_config.out_pin);

        module_data_t data = {};
        data.as_ack = is_polled;
        is_polled = false;

        prepare_result(&data);

        gpio_clear(adc_config.out_pin);

        /* Notify the application */
        callback(&data);

        /* Restart after delay */
        rtctimers_set_msg(&timer, 60 * adc_config.publish_period_min, &timer_msg, timer_pid);
    }

    return NULL;
}

static void set_period (int period) {
    adc_config.publish_period_min = period;
    save_config();

    /* Don't restart timer if new period is zero */
    if (adc_config.publish_period_min) {
        rtctimers_set_msg(&timer, 60 * adc_config.publish_period_min, &timer_msg, timer_pid);
        printf("[umdk-6adc] Period set to %d minutes\n", adc_config.publish_period_min);
    } else {
        puts("[umdk-6adc] Timer stopped");
    }
}

int umdk_6adc_shell_cmd(int argc, char **argv) {
    if (argc == 1) {
        puts ("6adc get - get results now");
        puts ("6adc send - get and send results now");
        puts ("6adc period <N> - set period to N minutes");
        puts ("6adc reset - reset settings to default");
        return 0;
    }
    
    char *cmd = argv[1];
	
    if (strcmp(cmd, "get") == 0) {
        module_data_t data = {};
        prepare_result(&data);
    }
    
    if (strcmp(cmd, "send") == 0) {
        is_polled = true;
		/* Send signal to publisher thread */
		msg_send(&timer_msg, timer_pid);
    }
    
    if (strcmp(cmd, "period") == 0) {
        char *val = argv[2];
        set_period(atoi(val));
    }
    
    if (strcmp(cmd, "reset") == 0) {
        reset_config();
        save_config();
    }
    
    return 1;
}

void umdk_6adc_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;
    init_config();

    init_gpio();
    init_adc(true);

    /* Create handler thread */
    char *stack = (char *) allocate_stack();
    if (!stack) {
    	puts("umdk-6adc: unable to allocate memory. Is too many modules enabled?");
    	return;
    }

    unwds_add_shell_command("6adc", "type '6adc' for commands list", umdk_6adc_shell_cmd);

    timer_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, timer_thread, NULL, "6ADC thread");

    /* Start publishing timer */
    rtctimers_set_msg(&timer, 60 * adc_config.publish_period_min, &timer_msg, timer_pid);
}

static void reply_ok(module_data_t *reply)
{
    reply->length = 2;
    reply->data[0] = UNWDS_6ADC_MODULE_ID;
    reply->data[1] = UMDK_6ADC_REPLY_OK;
}

static void reply_fail(module_data_t *reply)
{
    reply->length = 2;
    reply->data[0] = UNWDS_6ADC_MODULE_ID;
    reply->data[1] = UMDK_6ADC_REPLY_FAIL;
}

bool umdk_6adc_cmd(module_data_t *cmd, module_data_t *reply)
{
    if (cmd->length < 1) {
    	reply_fail(reply);
        return true;
    }

    umdk_6adc_cmd_t c = cmd->data[0];
    switch (c) {
        case UMDK_6ADC_CMD_SET_PERIOD: {
            if (cmd->length != 2) {
                reply_fail(reply);
                break;
            }

            uint8_t period = cmd->data[1];
            set_period(period);

            reply_ok(reply);
            break;
        }

        case UMDK_6ADC_CMD_POLL:
        	is_polled = true;

            /* Send signal to publisher thread */
            msg_send(&timer_msg, timer_pid);

            return false; /* Don't reply */

            break;

        case UMDK_6ADC_CMD_SET_GPIO: {
            uint8_t gpio = cmd->data[1];

            gpio_t pin = unwds_gpio_pin(gpio);
            if (pin) {
                adc_config.out_pin = pin;
                save_config();

                printf("[umdk-6adc] Out GPIO set to #%d\n", gpio);

                /* Re-initialize GPIO */
                init_gpio();

                reply_ok(reply);
                break;
            }

            reply_fail(reply);
            break;
        }

        case UMDK_6ADC_SET_LINES: {
            uint8_t lines = cmd->data[1];
            uint8_t line_num;

            for (line_num = 0; line_num < ADC_NUMOF; line_num++) {
                adc_config.adc_lines_en[line_num] = (lines >> line_num) & 1;

                if (adc_config.adc_lines_en[line_num])
                	printf("[umdk-6adc] Line #%d enabled\n", line_num);
            }

            save_config();

            /* Re-initialize ADC lines */
            init_adc(false);

            reply_ok(reply);
            break;
        }

        default:
            reply_fail(reply);
            break;
    }

    return true;
}

#ifdef __cplusplus
}
#endif
