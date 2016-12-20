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

/**
 * @brief Possible PWM frequencies table
 */
static uint32_t freq_table_hz[16] = {
	10,
	50,
	100,
	150,

	200,
	250,
	500,
	750,

	1000,
	2000,
	5000,
	10000,

	25000,
	50000,
	100000,
	150000,
};

static umdk_pwm_dev_t pwm_devs[UMDK_PWM_NUM_DEVS] = {
	{ PWM_0, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT },
	{ PWM_1, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT },
	{ PWM_2, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT },
};

static umdk_pwm_ch_t pwm_chs[UMDK_PWM_NUM_CH] = {
	{ PWM_0, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_0, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_0, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_0, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT },

	{ PWM_1, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_1, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT },

	{ PWM_2, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_2, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_2, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT },
	{ PWM_2, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT },
};

static uwnds_cb_t *callback;

void umdk_pwm_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
	(void)non_gpio_pin_map;

	callback = event_callback;

	for (int i = 0; i < UMDK_PWM_NUM_DEVS; i++) {
		umdk_pwm_dev_t *dev = &pwm_devs[i];

		pwm_init(dev->dev, dev->mode, dev->freq, dev->res);
	}

	for (int i = 0; i < UMDK_PWM_NUM_CH; i++) {
	  umdk_pwm_ch_t *ch = &pwm_chs[i];

	  pwm_set(ch->dev, ch->ch, ch->duty_cycle);
	}
}

static inline void update_pwm_freq(umdk_pwm_dev_t *dev, uint32_t freq) {
	dev->freq = freq;

	pwm_init(dev->dev, dev->mode, dev->freq, dev->res);
}

bool umdk_pwm_cmd(module_data_t *cmd, module_data_t *reply)
{
	/* Check minimum command length */
	if (cmd->length < 1)
		return false;

	umdk_pwm_cmd_t c = cmd->data[0];

	switch(c) {
	case UMDK_PWM_CMD_SET: {

		uint8_t ch_num = (cmd->data[1] >> 4) & 0x0F;
		uint32_t freq = freq_table_hz[cmd->data[1] & 0x0F];

		uint8_t value = cmd->data[2];

		/* Update corresponding PWM channel */
		umdk_pwm_ch_t *ch = &pwm_chs[ch_num];
		ch->duty_cycle = value;

		umdk_pwm_dev_t *dev = &pwm_devs[ch->dev];
		if (dev->freq != freq) {
			update_pwm_freq(dev, freq);
		}

		pwm_set(pwm_devs[ch->dev].dev, ch->ch, ch->duty_cycle);

		}
		break;
	}

  return false;
}


#ifdef __cplusplus
}
#endif
