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
 * @file		umdk-pwm.h
 * @brief       umdk-pwm driver module definitions
 * @author      Mikhail Perkov
 */
#ifndef UMDK_PWM_H
#define UMDK_PWM_H

#include "unwds-common.h"

#define UNWDS_PWM_MODULE_ID 14


void umdk_pwm_init(void);
bool umdk_pwm_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_PWM_H */
