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

static  uwnds_cb_t *callback;
/*static kernel_pid_t pwm_pid;*/

/*static void *pwm_handler(void *arg)
{
    (void)arg;

    msg_t msg;
    msg_t msg_queue[2];

    msg_init_queue(msg_queue, 2);

    while (1)
      {
        msg_receive(&msg);*/

        /* Notify the application */
       /* callback(&data);*/

/*        msg_send(&msg, pwm_pid);
    }

    return NULL;
}*/

void umdk_pwm_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
  (void)non_gpio_pin_map;

  callback = event_callback;

  pwm_init(PWM_0, PWM_LEFT, UNWDS_PWM_FREQ_DEFAULT, UNWDS_PWM_RES_DEFAULT);
  pwm_init(PWM_1, PWM_LEFT, UNWDS_PWM_FREQ_DEFAULT, UNWDS_PWM_RES_DEFAULT);
  pwm_init(PWM_2, PWM_LEFT, UNWDS_PWM_FREQ_DEFAULT, UNWDS_PWM_RES_DEFAULT);

  pwm_set(PWM_0, UNWDS_PWM_CH_0, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UNWDS_PWM_CH_1, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UNWDS_PWM_CH_2, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_0, UNWDS_PWM_CH_3, UNWDS_PWM_DUTY_DEFAULT);

  pwm_set(PWM_1, UNWDS_PWM_CH_0, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_1, UNWDS_PWM_CH_1, UNWDS_PWM_DUTY_DEFAULT);

  pwm_set(PWM_2, UNWDS_PWM_CH_0, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UNWDS_PWM_CH_1, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UNWDS_PWM_CH_2, UNWDS_PWM_DUTY_DEFAULT);
  pwm_set(PWM_2, UNWDS_PWM_CH_3, UNWDS_PWM_DUTY_DEFAULT);


  /* Create handler thread */
 /* char *stack = (char *) allocate_stack();
  if (!stack) {
  	puts("umdk-pwm: unable to allocate memory. Is too many modules enabled?");
  	return;
  }*/

 /* printf("[pwm] Current freq period: %d hour(s)\n", conf_counter.publish_period);*/

  /*pwm_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, pwm_handler, NULL, "pwm thread");*/

  /*msg_send(&msg, pwm_pid);*/
}



bool umdk_pwm_cmd(module_data_t *cmd, module_data_t *reply)
{
  return false;
}


#ifdef __cplusplus
}
#endif
