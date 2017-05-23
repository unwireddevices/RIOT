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
 * @file		unwds-common.h
 * @brief       common declarations for the unwired modules
 * @author      Eugene Ponomarev
 */
#ifndef UNWDS_COMMON_H_
#define UNWDS_COMMON_H_

#include <stdint.h>

#include "shell.h"
#include "shell_commands.h"
#include "periph/gpio.h"
#include "nvram.h"

#include "unwds-ids.h"

typedef uint8_t unwds_module_id_t;

/**
 * Modules NVRAM configuration.
 */

void unwds_setup_nvram_config(nvram_t *nvram_ptr, int base_addr, int block_size);

/**
 * @brief Reads data from NVRAM configuration for specified module
 *
 * @param	[in]	module_id	ID of the module
 * @param	[out]	*data_out	Data from NVRAM
 *
 * @return	true	reading success
 * @return	false	reading failed
 */
bool unwds_read_nvram_config(unwds_module_id_t module_id, uint8_t *data_out, uint8_t max_size);

/**
 * @brief Writes data to NVRAM configuration for specified module
 *
 * @param	[in]	module_id	ID of the module
 * @param	[in]	*data		Configuration data
 * @param	[in]	data_size	Size of configuration block data
 *
 * @return	true	writing success
 * @return	false	writing failed
 */
bool unwds_write_nvram_config(unwds_module_id_t module_id, uint8_t *data, size_t data_size);

/**
 * @brief Clears NVRAM configuration for specified module
 *
 * @param	[in]	module_id	ID of the module
 *
 * @return	true	cleared
 * @return	false	failed
 */
bool unwds_erase_nvram_config(unwds_module_id_t module_id);

/**
 * Stacks pool definitions.
 */
#define UNWDS_STACK_POOL_SIZE 5U
#define UNWDS_STACK_SIZE_BYTES (2048U)

uint8_t *allocate_stack(void);

#define UNWDS_MAX_MODULE_NAME 10
#define UNWDS_MAX_DATA_LEN 126

#if (CPU_MODEL == stm32l151cb)
    #define UNWDS_EEPROM_SIZE 4096
    #define UNWDS_CONFIG_STORAGE_SIZE 128
    #define UNWDS_STORAGE_BLOCKS_MAX 4
    #define UNWDS_MIN_CLEAN_BLOCKS 1
#else
    #define UNWDS_EEPROM_SIZE 8192
    #define UNWDS_CONFIG_STORAGE_SIZE 256
    #define UNWDS_STORAGE_BLOCKS_MAX 16
    #define UNWDS_MIN_CLEAN_BLOCKS 4
#endif

#define UNWDS_CONFIG_STORAGE_ADDR (UNWDS_EEPROM_SIZE - UNWDS_STORAGE_BLOCKS_MAX*UNWDS_CONFIG_STORAGE_SIZE)

/**
 * Shell commands
 */
#define UNWDS_SHELL_COMMANDS_MAX (10 + UNWDS_STACK_POOL_SIZE)
extern shell_command_t shell_commands[UNWDS_SHELL_COMMANDS_MAX];

/**
 * @brief Holds data transferred to/from module
 */
typedef struct {
	uint8_t length;
	uint8_t data[UNWDS_MAX_DATA_LEN];

	int16_t rssi;

	bool as_ack;	/**< This data could be sent as ACK for downlink command */
} module_data_t;

typedef void (uwnds_cb_t)(module_data_t *msg);

typedef struct {
	unwds_module_id_t module_id;
	char name[UNWDS_MAX_MODULE_NAME];
	void (*init_cb)(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
	bool (*cmd_cb)(module_data_t *data, module_data_t *reply);

	bool (*cmb_broadcast_cb)(module_data_t *data, module_data_t *reply);
} unwd_module_t;

void unwds_init_modules(uwnds_cb_t *event_callback);
bool unwds_send_to_module(unwds_module_id_t modid, module_data_t *data, module_data_t *reply);
bool unwds_send_broadcast(unwds_module_id_t modid, module_data_t *data, module_data_t *reply);

void unwds_add_shell_command(char *name, char *desc, void* handler);

uint32_t * unwds_get_enabled(void);
void unwds_set_enabled(uint32_t *ability);

char *unwds_get_module_name(unwds_module_id_t modid);

void unwds_list_modules(uint32_t *ability, bool enabled_only);

bool unwds_is_pin_occupied(uint32_t pin);
bool unwds_is_module_exists(unwds_module_id_t modid);

uint64_t unwds_get_ability_mask(unwds_module_id_t modid);

int unwds_modid_by_name(char *name);

gpio_t unwds_gpio_pin(int pin);
int unwds_gpio_pins_total(void);

void int_to_float_str(char *buf, int decimal, uint8_t precision);

bool unwds_read_nvram_storage(unwds_module_id_t module_id, uint8_t *data_out, uint8_t size);
bool unwds_write_nvram_storage(unwds_module_id_t module_id, uint8_t *data, size_t data_size);

void ungets(char *str);

#endif /* UNWDS_COMMON_H_ */
