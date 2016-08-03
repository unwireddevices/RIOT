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
#ifndef UNWDS_GPIO_H
#define UNWDS_GPIO_H

#include "unwds-common.h"

void unwds_gpio_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool unwds_gpio_cmd(int argc, char argv[UNWDS_MAX_PARAM_COUNT][UNWDS_MAX_PARAM_LEN], char *reply);

#endif /* UNWDS_GPIO_H */
