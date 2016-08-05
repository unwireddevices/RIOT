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
#ifndef UNWIRED_MODULES_UNWDS_COMMON_INCLUDE_UNWDS_COMMON_H_
#define UNWIRED_MODULES_UNWDS_COMMON_INCLUDE_UNWDS_COMMON_H_

#include <stdint.h>

#define UNWDS_MAX_MODULE_NAME 10

#define UNWDS_PARAM_DELIM ','
#define UNWDS_MAX_PARAM_COUNT 4
#define UNWDS_MAX_PARAM_LEN 64

#define UNWDS_MAX_REPLY_LEN 128

typedef void (uwnds_cb_t)(char *msg);

typedef struct {
	char name[UNWDS_MAX_MODULE_NAME];

	void (*init_cb)(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
	bool (*cmd_cb)(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply);

	uint64_t ability_mask;
} unwd_module_t;

void unwds_init(uwnds_cb_t *callback);
bool unwds_command(char *command, char *reply);

uint64_t unwds_get_ability(void);

bool unwds_is_pin_occupied(uint32_t pin);

#endif /* UNWIRED_MODULES_UNWDS_COMMON_INCLUDE_UNWDS_COMMON_H_ */
