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


#include "periph/gpio.h"
#include "periph/pwm.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-pwm.h"

#include "thread.h"


static  uwnds_cb_t *callback;

void umdk_pwm_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
  (void)non_gpio_pin_map;

  callback = event_callback;

  pwm_init(PWM_0, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT);
  pwm_init(PWM_1, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT);
  pwm_init(PWM_2, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT);

  pwm_set(PWM_0, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT);

  pwm_set(PWM_1, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_1, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT);

  pwm_set(PWM_2, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT);

  pwm_set(PWM_0, UMDK_PWM_CH_0, 50);

  pwm_start(PWM_0);

  printf("[pwm] start");

}



bool umdk_pwm_cmd(module_data_t *cmd, module_data_t *reply)
{
  return false;
}


#ifdef __cplusplus
}
#endif
