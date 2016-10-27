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
#include "umdk-acc.h"
#include "umdk-lmt01.h"
#include "umdk-uart.h"
#include "umdk-sht21.h"
#include "umdk-pir.h"
#include "umdk-6adc.h"
#include "umdk-lps331.h"

/**
 * @brief Bitmap of occupied pins that cannot be used as gpio in-out
 */
static uint32_t non_gpio_pin_map;

/**
 * @brief Device abilities enabled by the used modules mask
 */
static uint64_t ability_map;

static const unwd_module_t modules[] = {
    { UNWDS_GPIO_MODULE_ID, "gpio", unwds_gpio_init, unwds_gpio_cmd, 1 << 1 },
    { UNWDS_4BTN_MODULE_ID, "4btn", umdk_4btn_init, umdk_4btn_cmd, 1 << 2 },
    { UNWDS_GPS_MODULE_ID, "gps", umdk_gps_init, umdk_gps_cmd, 1 << 3 },
	/*
    { UNWDS_TEMP_MODULE_ID, "temp", umdk_temp_init, umdk_temp_cmd, 1 << 4 },
	{ UNWDS_ACC_MODULE_ID, "acc", umdk_acc_init, umdk_acc_cmd, 1 << 5 },*/
	{ UNWDS_LMT01_MODULE_ID, "lmt01", umdk_lmt01_init, umdk_lmt01_cmd, 1 << 6 },
	{ UNWDS_UART_MODULE_ID, "uart", umdk_uart_init, umdk_uart_cmd, 1 << 7 },
	{ UNWDS_SHT21_MODULE_ID, "sht21", umdk_sht21_init, umdk_sht21_cmd, 1 << 8 },
	{ UNWDS_PIR_MODULE_ID, "pir", umdk_pir_init, umdk_pir_cmd, 1 << 9 },
	{ UNWDS_6ADC_MODULE_ID, "6adc", umdk_6adc_init, umdk_6adc_cmd, 1 << 10 },
	{ UNWDS_LPS331_MODULE_ID, "lps331", umdk_lps331_init, umdk_lps331_cmd, 1 << 11 },

    { 0, "", NULL, NULL },
};

/**
 * Stacks pool.
 */
static uint8_t stacks_pool[UNWDS_STACK_POOL_SIZE][UNWDS_STACK_SIZE_BYTES];
static uint8_t stacks_allocated = 0;

uint8_t *allocate_stack(void) {
	/* Stacks pool is full */
	if (stacks_allocated == UNWDS_STACK_POOL_SIZE)
		return NULL;

	return stacks_pool[stacks_allocated++];
}

void unwds_init_modules(uwnds_cb_t *event_callback)
{
    int i = 0;

    /* Pre-initialize I2C */
    i2c_init_master(I2C_0, I2C_SPEED_NORMAL);

    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
    	if (ability_map & modules[i].ability_mask) {
    		printf("[unwds] initializing \"%s\" module...\n", modules[i].name);
        	modules[i].init_cb(&non_gpio_pin_map, event_callback);
    	}

        i++;
    }
}

static unwd_module_t *find_module(unwds_module_id_t modid) {
	int i = 0;
    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
    	if (modules[i].module_id == modid)
    		return (unwd_module_t *) &modules[i];

    	i++;
    }

    return NULL;
}

void unwds_list_modules(uint64_t ability, bool enabled_only) {
	int i = 0;
	int modcount = 0;
    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
    	bool enabled = (ability & modules[i].ability_mask);
    	unwds_module_id_t modid = modules[i].module_id;

    	if (enabled_only && !enabled) {
    		i++;
    		continue;
    	}

    	modcount++;
    	printf("[%s] %s (id: %d)\n", (enabled) ? "+" : "-", modules[i].name, modid);

    	i++;
    }

    if (!modcount)
    	puts("<no modules enabled>");
}

void unwds_set_ability(uint64_t ability) {
	ability_map = ability;
}

uint64_t unwds_get_ability_mask(unwds_module_id_t modid) {
	unwd_module_t *module = find_module(modid);
	if (!module)
		return 0;

	return module->ability_mask;
}

bool unwds_is_module_exists(unwds_module_id_t modid) {
	return find_module(modid) != NULL;
}

bool unwds_send_to_module(unwds_module_id_t modid, module_data_t *data, module_data_t *reply)
{

	unwd_module_t *module = find_module(modid);
	if (!module)
		return false;

	return module->cmd_cb(data, reply);
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
