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

/**
 * Stacks pool definitions.
 */
#define UNWDS_STACK_POOL_SIZE 5U
#define UNWDS_STACK_SIZE_BYTES (2048U)

uint8_t *allocate_stack(void);

#define UNWDS_MAX_MODULE_NAME 10
#define UNWDS_MAX_DATA_LEN 126

/**
 * @brief Holds data transferred to/from module
 */
typedef struct {
	uint8_t length;
	uint8_t data[UNWDS_MAX_DATA_LEN];
} module_data_t;

typedef void (uwnds_cb_t)(module_data_t *msg);

typedef uint8_t unwds_module_id_t;

typedef struct {
	unwds_module_id_t module_id;
	char name[UNWDS_MAX_MODULE_NAME];
	void (*init_cb)(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
	bool (*cmd_cb)(module_data_t *data, module_data_t *reply);

	uint64_t ability_mask;
} unwd_module_t;

void unwds_init_modules(uwnds_cb_t *event_callback);
bool unwds_send_to_module(unwds_module_id_t modid, module_data_t *data, module_data_t *reply);

uint64_t unwds_get_ability(void);
void unwds_set_ability(uint64_t ability);

void unwds_list_modules(uint64_t ability, bool enabled_only);

bool unwds_is_pin_occupied(uint32_t pin);
bool unwds_is_module_exists(unwds_module_id_t modid);

uint64_t unwds_get_ability_mask(unwds_module_id_t modid);

#endif /* UNWDS_COMMON_H_ */
