/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
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

typedef uint8_t unwds_module_id_t;

/**
 * Modules NVRAM configuration.
 */

void unwds_setup_nvram_config(int base_addr, int block_size);

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

uint8_t *allocate_stack_name(uint32_t stack_size, const char* caller_name);

#define allocate_stack(stack_size) allocate_stack_name(stack_size, __func__)

#define UNWDS_MAX_MODULE_NAME 15
#define UNWDS_MAX_DATA_LEN 126

#define UNWDS_STORAGE_BLOCKS_MAX 16

#define UNWDS_MODULE_NO_DATA    0
#define UNWDS_MODULE_HAS_DATA   1
#define UNWDS_MODULE_NOT_FOUND  255

#if defined(UNWDS_BUILD_MINIMAL)
    #define UNWDS_SHELL_COMMANDS_MAX (12)
#else
    #define UNWDS_SHELL_COMMANDS_MAX (20)
#endif

/**
 * Shell commands
 */
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
	void (*init_cb)(uwnds_cb_t *event_callback);
	bool (*cmd_cb)(module_data_t *data, module_data_t *reply);

	bool (*cmb_broadcast_cb)(module_data_t *data, module_data_t *reply);
} unwd_module_t;

typedef enum {
    UNWDS_BOOT_NORMAL_MODE = 0,
    UNWDS_BOOT_SAFE_MODE = 1,
    UNWDS_BOOT_MODULES_FAILED = 2,
} boot_modes_t;

extern bool lorawan_busy;

void unwds_init_modules(uwnds_cb_t *event_callback);
int unwds_send_to_module(unwds_module_id_t modid, module_data_t *data, module_data_t *reply);
bool unwds_send_broadcast(unwds_module_id_t modid, module_data_t *data, module_data_t *reply);

void unwds_add_shell_command(char *name, char *desc, void* handler);

uint8_t *unwds_get_enabled(void);
void unwds_set_enabled(uint8_t *ability);

char *unwds_get_module_name(unwds_module_id_t modid);

void unwds_list_modules(uint8_t *ability, bool enabled_only);

bool unwds_is_module_exists(unwds_module_id_t modid);
bool unwds_is_module_enabled(unwds_module_id_t modid);

int unwds_modid_by_name(char *name);

gpio_t unwds_gpio_pin(int pin);
int unwds_gpio_pins_total(void);

void int_to_float_str(char *buf, int decimal, uint8_t precision);

/* converts number to BE, sign-and-magnitude format */
void convert_to_be_sam(void *ptr, size_t size);

/* converts number to BE, sign-and-magnitude format */
void convert_from_be_sam(void *ptr, size_t size);

bool unwds_read_nvram_storage(unwds_module_id_t module_id, uint8_t *data_out, size_t size);
bool unwds_write_nvram_storage(unwds_module_id_t module_id, uint8_t *data, size_t data_size);

void blink_led(gpio_t led);
void print_logo(void);

#endif /* UNWDS_COMMON_H_ */
