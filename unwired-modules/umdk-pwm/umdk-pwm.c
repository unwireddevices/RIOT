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
 * @file	umdk-pwm.c
 * @brief       umdk-pwm module implementation
 * @author      Mikhail Perkov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "lpm.h"

#include "periph/gpio.h"
#include "periph/pwm.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-pwm.h"

#include "thread.h"
#include "xtimer.h"
#include "rtc-timers.h"

//static kernel_pid_t handler_pid;



void umdk_pwm_init(void)
{
  pwm_init(PWM_0_EN, PWM_LEFT, 100, 10);
}



bool umdk_pwm_cmd(module_data_t *cmd, module_data_t *reply)
{
  return false;
}


#ifdef __cplusplus
}
#endif
