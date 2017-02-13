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

#include "nvram.h"

#include "unwds-common.h"

#include "unwds-gpio.h"
#include "umdk-4btn.h"
#include "umdk-gps.h"
#include "umdk-lm75.h"
#include "umdk-lsm6ds3.h"
#include "umdk-lmt01.h"
#include "umdk-uart.h"
#include "umdk-sht21.h"
#include "umdk-pir.h"
#include "umdk-6adc.h"
#include "umdk-lps331.h"
#include "umdk-4counter.h"
#include "umdk-rssiecho.h"
#include "umdk-pwm.h"
#include "umdk-opt3001.h"
#include "umdk-dali.h"
#include "umdk-bme280.h"

/**
 * @brief Bitmap of occupied pins that cannot be used as gpio in-out
 */
static uint32_t non_gpio_pin_map;

/**
 * @brief Bitmap of enabled modules
 */
static uint64_t enabled_bitmap;

static const unwd_module_t modules[] = {

#ifdef umdk_gpio
    { UNWDS_GPIO_MODULE_ID, "gpio", unwds_gpio_init, unwds_gpio_cmd, unwds_gpio_broadcast },
#endif
#ifdef umdk_4btn
    { UNWDS_4BTN_MODULE_ID, "4btn", umdk_4btn_init, umdk_4btn_cmd, NULL },
#endif
#ifdef umdk_gps
    { UNWDS_GPS_MODULE_ID, "gps", umdk_gps_init, umdk_gps_cmd, NULL },
#endif
#ifdef umdk_lsm6ds3
	{ UNWDS_LSM6DS3_MODULE_ID, "lsm6ds3", umdk_lsm6ds3_init, umdk_lsm6ds3_cmd, NULL },
#endif
#ifdef umdk_lm75
    { UNWDS_LM75_MODULE_ID, "lm75", umdk_lm75_init, umdk_lm75_cmd, NULL },
#endif
#ifdef umdk_lmt01
	{ UNWDS_LMT01_MODULE_ID, "lmt01", umdk_lmt01_init, umdk_lmt01_cmd, NULL},
#endif
#ifdef umdk_uart
	{ UNWDS_UART_MODULE_ID, "uart", umdk_uart_init, umdk_uart_cmd, NULL },
#endif
#ifdef umdk_sht21
	{ UNWDS_SHT21_MODULE_ID, "sht21", umdk_sht21_init, umdk_sht21_cmd, NULL },
#endif
#ifdef umdk_pir
	{ UNWDS_PIR_MODULE_ID, "pir", umdk_pir_init, umdk_pir_cmd, NULL },
#endif
#ifdef umdk_6adc
	{ UNWDS_6ADC_MODULE_ID, "6adc", umdk_6adc_init, umdk_6adc_cmd, NULL },
#endif
#ifdef umdk_lps331
	{ UNWDS_LPS331_MODULE_ID, "lps331", umdk_lps331_init, umdk_lps331_cmd, NULL },
#endif
#ifdef umdk_4counter
	{ UNWDS_4COUNTER_MODULE_ID, "4counter", umdk_4counter_init, umdk_4counter_cmd, NULL },
#endif
#ifdef umdk_rssiecho
	{ UNWDS_RSSIECHO_MODULE_ID, "echo", umdk_rssiecho_init, umdk_rssiecho_cmd, NULL },
#endif
#ifdef umdk_pwm
	{ UNWDS_PWM_MODULE_ID, "pwm", umdk_pwm_init, umdk_pwm_cmd, NULL },
#endif
#ifdef umdk_opt3001
	{ UNWDS_OPT3001_MODULE_ID, "opt3001", umdk_opt3001_init, umdk_opt3001_cmd, NULL },
#endif
#ifdef umdk_dali
	{ UNWDS_DALI_MODULE_ID, "dali", umdk_dali_init, umdk_dali_cmd, NULL },
#endif
#ifdef umdk_bme280
	{ UNWDS_BME280_MODULE_ID, "bme280", umdk_bme280_init, umdk_bme280_cmd, NULL },
#endif
    { 0, "", NULL, NULL },
};

/**
 * NVRAM config.
 */
static nvram_t *nvram = NULL;
static int nvram_config_block_size = 0;
static int nvram_config_base_addr = 0;

void unwds_setup_nvram_config(nvram_t *nvram_ptr, int base_addr, int block_size) {
	nvram = nvram_ptr;
	nvram_config_base_addr = base_addr;
	nvram_config_block_size = block_size;
}

bool unwds_read_nvram_config(unwds_module_id_t module_id, uint8_t *data_out, uint8_t max_size) {
	/* All configuration blocks has the same size */
	int addr = nvram_config_base_addr + module_id * nvram_config_block_size;

	/* Either max_size bytes or full block */
	int size = (max_size < nvram_config_block_size) ? max_size : nvram_config_block_size;

	/* Read NVRAM block */
	if (nvram->read(nvram, data_out, addr, size) < 0)
		return false;

	return true;
}

bool unwds_write_nvram_config(unwds_module_id_t module_id, uint8_t *data, size_t data_size) {
	if (data_size > nvram_config_block_size)
		return false;

	/* All configuration blocks has the same size */
	int addr = nvram_config_base_addr + module_id * nvram_config_block_size;

	/* Write NVRAM block */
	if (nvram->write(nvram, data, addr, nvram_config_block_size) < 0)
		return false;

	return true;
}

bool unwds_erase_nvram_config(unwds_module_id_t module_id) {
	/* All configuration blocks has the same size */
	int addr = nvram_config_base_addr + module_id * nvram_config_block_size;

	/* Write NVRAM block */
	if (nvram->clearpart(nvram, addr, nvram_config_block_size) < 0)
		return false;

	return true;
}

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

	/* Initialize modules */
    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
    	if (enabled_bitmap & (1 << modules[i].module_id)) {	/* Module enabled */
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

void unwds_list_modules(uint64_t enabled_mods, bool enabled_only) {
	int i = 0;
	int modcount = 0;
    while (modules[i].init_cb != NULL && modules[i].cmd_cb != NULL) {
    	bool enabled = (enabled_mods & (1 << modules[i].module_id));
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

void unwds_set_enabled(uint64_t enabled_mods) {
	enabled_bitmap = enabled_mods;
}

char *unwds_get_module_name(unwds_module_id_t modid) {
	unwd_module_t *module = find_module(modid);
	if (!module)
		return NULL;

	return module->name;	
}

bool unwds_is_module_exists(unwds_module_id_t modid) {
	return find_module(modid) != NULL;
}

bool unwds_send_broadcast(unwds_module_id_t modid, module_data_t *data, module_data_t *reply)
{
	unwd_module_t *module = find_module(modid);
	if (!module)
		return false;

	if (module->cmb_broadcast_cb != NULL)
		return module->cmb_broadcast_cb(data, reply);

	return false;
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

uint64_t unwds_get_enabled(void)
{
    return enabled_bitmap;
}

#ifdef __cplusplus
}
#endif
