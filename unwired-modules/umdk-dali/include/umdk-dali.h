/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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

#define UMDK_DALI_NUM_CHANNELS 4

#define UMDK_DALI_1 UNWD_GPIO_4
#define UMDK_DALI_2 UNWD_GPIO_5
#define UMDK_DALI_3 UNWD_GPIO_6
#define UMDK_DALI_4 UNWD_GPIO_7

#define UMDK_DALI_LENGTH_PACK 19
#define UMDK_DALI_FREQUENCY_HZ 2400000
#define UMDK_DALI_TIME_TX_PACK_USEC 416
#define UMDK_DALI_TIME_MANCHESTER_USEC (UMDK_DALI_TIME_TX_PACK_USEC / 2)

#define _BIT(n) (1<<n)
#define _CHKBIT(reg,n) (reg & _BIT(n))

void umdk_dali_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_dali_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_DALI_H */
