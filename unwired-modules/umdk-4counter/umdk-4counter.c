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
 * @file	umdk-4counter.c
 * @brief       umdk-4counter module implementation
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

#include "board.h"

#include "unwds-common.h"
#include "umdk-4counter.h"

#include "thread.h"
#include "xtimer.h"
#include "rtc-timers.h"


static kernel_pid_t handler_pid;
static uint32_t last_pressed[4] = {};

static uwnds_cb_t *callback;
static rtctimer_t timer;

static msg_t handler_msg = {};

static struct  {
	uint8_t  is_valid;
	uint32_t count_value[4];
	uint8_t publish_period;
} conf_counter;


static inline void save_config(void)
{
  conf_counter.is_valid = 1;
  unwds_write_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter));
}

static void *handler(void *arg)
{
    msg_t msg;
    msg_t msg_queue[2];
    msg_init_queue(msg_queue, 2);

    while (1)
      {
        msg_receive(&msg);

	lpm_prevent_sleep = 1;

	module_data_t data;
	data.length = 17;
	data.data[0] = UNWDS_4COUNTER_MODULE_ID;

	uint32_t * tmp = (uint32_t *)(&data.data[1]);

	*(tmp + 0)  = conf_counter.count_value[0];
	*(tmp + 1)  = conf_counter.count_value[1];
	*(tmp + 2)  = conf_counter.count_value[2];
	*(tmp + 3)  = conf_counter.count_value[3];

	save_config();	// Save values by NVRAM

	callback(&data);

	/* Restart timer after delay */
	if (conf_counter.publish_period)
	  rtctimers_set_msg(&timer, 3600 * conf_counter.publish_period, &handler_msg, handler_pid);
	  /* Sleep */
	lpm_prevent_sleep = 0;
    }
	return NULL;
}

static void counter_int(void *arg)
{
  uint8_t btn_num = ((int) arg) - 1;
  /* Wake up */
  lpm_prevent_sleep = 1;

  uint32_t now = xtimer_now();
  /* Timer overflows every ~71 minutes */
  uint32_t overflow = 0;
  if (last_pressed[btn_num] > now) {
      overflow = UINT32_MAX - last_pressed[btn_num];
  }

  /* Don't accept a press of current button if it did occur earlier than last press plus debouncing time */
  if (overflow + now - last_pressed[btn_num] <= UMDK_4COUNT_DEBOUNCE_TIME_MS * 1000) {
      printf("[4counter] Counting %d rejected\n", (btn_num + 1));
      return;
  }

  /* Counting value of each sensors */
  conf_counter.count_value[btn_num]++;

  last_pressed[btn_num] = now;
  /* Sleep */
  lpm_prevent_sleep = 0;
}

void umdk_4counter_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
  (void) non_gpio_pin_map;

  conf_counter.publish_period = UMDK_4COUNT_PUBLISH_PERIOD_MIN;

  callback = event_callback;

  /* Initialize interrupts */
  gpio_init_int(UMDK_4COUNT_1, GPIO_IN_PU, GPIO_FALLING, counter_int, (void *) 1);
  gpio_init_int(UMDK_4COUNT_2, GPIO_IN_PU, GPIO_FALLING, counter_int, (void *) 2);
  gpio_init_int(UMDK_4COUNT_3, GPIO_IN_PU, GPIO_FALLING, counter_int, (void *) 3);
  gpio_init_int(UMDK_4COUNT_4, GPIO_IN_PU, GPIO_FALLING, counter_int, (void *) 4);

  /* Create handler thread */
  char *stack = (char *) allocate_stack();
  if (!stack) {
	puts("umdk-4counter: unable to allocate memory. Is too many modules enabled?");
	return;
  }

  /* Load config from NVRAM */
  unwds_read_nvram_config(UNWDS_4COUNTER_MODULE_ID, (uint8_t *) &conf_counter, sizeof(conf_counter));

  printf("[4counter] Current publish period: %d hour(s)\n", conf_counter.publish_period);

  handler_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, handler, NULL, "4counter thread");

   /* Start publishing timer */
   rtctimers_set_msg(&timer, 3600 * conf_counter.publish_period, &handler_msg, handler_pid);
}


bool umdk_4counter_cmd(module_data_t *cmd, module_data_t *reply)
{
 if (cmd->length < 1)
	  return false;

  umdk_4counter_cmd_t c = cmd->data[0];
  switch (c) {
  case UMDK_4COUNT_CMD_SET_PERIOD: {
      if (cmd->length != 2)
	      return false;

      uint8_t period = cmd->data[1];
      rtctimers_remove(&timer);

      conf_counter.publish_period = period;
      save_config();

      /* Don't restart timer if new period is zero */
      if ((conf_counter.publish_period) && (conf_counter.publish_period < (UMDK_4COUNT_PUBLISH_PERIOD_MAX + 1))) {
	  rtctimers_set_msg(&timer, 3600 * conf_counter.publish_period, &handler_msg, handler_pid);
	      printf("[4counter] Period set to %d hour (s)\n", conf_counter.publish_period);
      } else
	      puts("[4counter] Timer stopped");

      reply->length = 4;
      reply->data[0] = UNWDS_4COUNTER_MODULE_ID;
      reply->data[1] = 'o';
      reply->data[2] = 'k';
      reply->data[3] = '\0';

      break;
    }

  case UMDK_4COUNTER_CMD_POLL:
	  /* Send values to publisher thread */
      msg_send(&handler_msg, handler_pid);

      return false; /* Don't reply */

      break;

    default:
      break;
  }

  return true;
}

#ifdef __cplusplus
}
#endif
