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
 * @author		Eugene Ponomarev
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-pwm.h"

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
	125000,
};

static umdk_pwm_dev_t pwm_devs[UMDK_PWM_NUM_DEVS] = {
	{ PWM_0, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT, false },
	{ PWM_1, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT, false },
	{ PWM_2, PWM_LEFT, UMDK_PWM_FREQ_DEFAULT, UMDK_PWM_RES_DEFAULT, false },
};

static umdk_pwm_ch_t pwm_chs[UMDK_PWM_NUM_CH] = {
	{ 0, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT },
	{ 0, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT },
	{ 0, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT },
	{ 0, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT },

	{ 1, UMDK_PWM_CH_0, UMDK_PWM_DUTY_DEFAULT },
	{ 1, UMDK_PWM_CH_1, UMDK_PWM_DUTY_DEFAULT },

	{ 2, UMDK_PWM_CH_2, UMDK_PWM_DUTY_DEFAULT },
	{ 2, UMDK_PWM_CH_3, UMDK_PWM_DUTY_DEFAULT },
};

static uwnds_cb_t *callback;

void umdk_pwm_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
	(void)non_gpio_pin_map;

	callback = event_callback;

	for (int i = 0; i < UMDK_PWM_NUM_DEVS; i++) {
		umdk_pwm_dev_t *dev = &pwm_devs[i];

		printf("[umdk-pwm] Initializing PWM#%d with frequency %d Hz and resolution up to %d\n", dev->dev, (int) dev->freq, dev->res);

		pwm_init(dev->dev, dev->mode, dev->freq, dev->res);
	}
}

static inline void update_pwm_freq(umdk_pwm_dev_t *dev, uint32_t freq) {
	dev->freq = freq;

	pwm_stop(dev->dev);
	pwm_init(dev->dev, dev->mode, dev->freq, dev->res);
}

static void set_pwm_value(umdk_pwm_dev_t *dev, umdk_pwm_ch_t *ch, uint8_t value) {
	ch->duty_cycle = value;

	/*
	 * Check that device for corresponding channel is need to be started or can be stopped.
	 * We could stop the PWM device if all it's channels are having default duty cycle
	 */
	bool need_to_start = !dev->is_started;
	bool can_be_stopped = true;
	for (int i = 0; i < UMDK_PWM_NUM_CH; i++) {
	  umdk_pwm_ch_t *ch = &pwm_chs[i];

	  /* Check channels in current PWM device */
	  if ((&pwm_devs[ch->dev])->dev == dev->dev) {
		  /* Device can't be stopped if it has channels with non-default duty cycle */
		  if (ch->duty_cycle != UMDK_PWM_DUTY_DEFAULT) {
			  can_be_stopped = false;
		  }
	  }
	}

	/* Start or stop current PWM device */
	if (need_to_start) {
		pwm_start(dev->dev);
		dev->is_started = true;

		printf("[umdk-pwm] PWM device #%d is started\n", ch->dev);
	} else
	if (can_be_stopped) {
		pwm_stop(dev->dev);
		dev->is_started = false;

		printf("[umdk-pwm] PWM device #%d is stopped\n", ch->dev);
	}

	/* Set value for the current channel in current PWM device if it's running */
	if (dev->is_started) {
		printf("[umdk-pwm] Setting pwm #%d channel #%d to %d\n", dev->dev, ch->ch, ch->duty_cycle);
		pwm_set(dev->dev, ch->ch, ch->duty_cycle);
	}
}

bool umdk_pwm_cmd(module_data_t *cmd, module_data_t *reply)
{
	/* Check minimum command length */
	if (cmd->length < 2)
		return false;

	umdk_pwm_cmd_t c = cmd->data[0];

	switch(c) {
	case UMDK_PWM_CMD_SET: {
		uint8_t ch_num = (cmd->data[1] >> 4) & 0x0F;
		uint8_t freq_num = cmd->data[1] & 0x0F;

		uint32_t freq = freq_table_hz[freq_num];
		uint8_t value = cmd->data[2];

		if (ch_num >= UMDK_PWM_NUM_CH) {
			printf("[umdk-pwm] Invalid channel selected: %d >= %d\n", ch_num, UMDK_PWM_NUM_CH);
			return false;
		}

		/* Update corresponding PWM channel */
		umdk_pwm_ch_t *ch = &pwm_chs[ch_num];
		umdk_pwm_dev_t *dev = &pwm_devs[ch->dev];

		if (dev->freq != freq) {
			printf("[umdk-pwm] Updating PWM device #%d frequency from %d to %d Hz\n", dev->dev, (int) dev->freq, (int) freq);
			update_pwm_freq(dev, freq);
		}

		printf("[umdk-pwm] Setting PWM#%d ch: %d to %d/%d with frequency %d Hz\n",
				dev->dev, ch_num, value, dev->res, (int) freq);

		set_pwm_value(dev, ch, value);

		}
		break;
	}

  return false;
}


#ifdef __cplusplus
}
#endif
