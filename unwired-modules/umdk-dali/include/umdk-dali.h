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
 * @file		umdk-dali.h
 * @brief       umdk-dali driver module definitions
 * @author      Mikhail Perkov

 */
#ifndef UMDK_DALI_H
#define UMDK_DALI_H

#include "unwds-common.h"
#include "periph/pwm.h"

#define UNWDS_DALI_MODULE_ID 16

#define UMDK_DALI_CH_0 0
#define UMDK_DALI_CH_1 1
#define UMDK_DALI_CH_2 2
#define UMDK_DALI_CH_3 3


/**
 * @brief UMDK-DALI module commands list
 */


void umdk_dali_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_dali_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_DALI_H */
