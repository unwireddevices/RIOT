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
 * @file	    umdk-irblaster.h
 * @brief       umdk-irblaster module definitions
 * @author      Oleg Artamonov
 */
#ifndef UMDK_IR_H
#define UMDK_IR_H

#include "unwds-common.h"

#include "periph/pwm.h"

#define UMDK_IR_0 0
#define UMDK_IR_1 1
#define UMDK_IR_2 2

#define UMDK_IR_CH_0 0
#define UMDK_IR_CH_1 1
#define UMDK_IR_CH_2 2
#define UMDK_IR_CH_3 3

#define UMDK_IR_DUTY_DEFAULT 50
#define UMDK_IR_FREQ_DEFAULT (38000U)
#define UMDK_IR_RES_DEFAULT 100

#define UMDK_IR_DUTY_MAX 100
#define UMDK_IR_FREQ_MAX (100000U)

#define UMDK_IR_NUM_DEVS 3
#define UMDK_IR_NUM_CH 8

#define UMDK_IR_NUM_CH_MAX 4
#define UMDK_IR_0_NUM_CH_MAX 4
#define UMDK_IR_1_NUM_CH_MAX 2
#define UMDK_IR_2_NUM_CH_MAX 2

#define UMDK_IR_STATUS_DEFAULT 1
#define UMDK_IR_CH_TURN_ON 1
#define UMDK_IR_CH_TURN_OFF 0

/**
 * @brief PWM channel structure
 */
typedef struct {
    uint8_t ch;             /**< PWM channel number */
    uint8_t status;	/**< Status of work PWM channel */
    uint16_t duty_cycle;    /**< Current channel duty cycle */
} umdk_irblaster_ch_t;

/**
 * @brief PWM device structure
 */
typedef struct {
    pwm_t dev;          /**< PWM device number*/

    uint8_t num_chan;	/**< Number of channels of the PWM device*/
    pwm_mode_t mode;    /**< PWM device mode */
    uint32_t freq;      /**< PWM device frequency */
    uint16_t res;       /**< PWM device resolution */

    umdk_irblaster_ch_t pwm_chs[UMDK_IR_NUM_CH_MAX];	/**< Configuration of PWM channels*/

    bool is_started;    /**< PWM device is running */
} umdk_irblaster_dev_t;


/**
 * @brief UMDK-PWM module commands list
 */
typedef enum {
    UMDK_IR_CMD_SEND = 0, /**< Sets frequency and duty cycle for specified PWM channel  */
} umdk_irblaster_cmd_t;

void umdk_irblaster_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_irblaster_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_IR_H */
