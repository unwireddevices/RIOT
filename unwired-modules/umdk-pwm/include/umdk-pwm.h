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
 * @author		Eugene Ponomarev
 */
#ifndef UMDK_PWM_H
#define UMDK_PWM_H

#include "unwds-common.h"

#include "periph/pwm.h"

#define UMDK_PWM_0 0
#define UMDK_PWM_1 1
#define UMDK_PWM_2 2

#define UMDK_PWM_CH_0 0
#define UMDK_PWM_CH_1 1
#define UMDK_PWM_CH_2 2
#define UMDK_PWM_CH_3 3

#define UMDK_PWM_DUTY_DEFAULT 0
#define UMDK_PWM_FREQ_DEFAULT (1000U)
#define UMDK_PWM_RES_DEFAULT 100

#define UMDK_PWM_DUTY_MAX 100
#define UMDK_PWM_FREQ_MAX (100000U)

#define UMDK_PWM_NUM_DEVS 3
#define UMDK_PWM_NUM_CH 8

#define UMDK_PWM_NUM_CH_MAX 4
#define UMDK_PWM_0_NUM_CH_MAX 4
#define UMDK_PWM_1_NUM_CH_MAX 2
#define UMDK_PWM_2_NUM_CH_MAX 2

#define UMDK_PWM_STATUS_DEFAULT 0
#define UMDK_PWM_CH_TURN_ON 1
#define UMDK_PWM_CH_TURN_OFF 0

/**
 * @brief PWM channel structure
 */
typedef struct {
    uint8_t ch;             /**< PWM channel number */
    uint8_t status;	/**< Status of work PWM channel */
    uint16_t duty_cycle;    /**< Current channel duty cycle */
} umdk_pwm_ch_t;

/**
 * @brief PWM device structure
 */
typedef struct {
    pwm_t dev;          /**< PWM device number*/

    uint8_t num_chan;	/**< Number of channels of the PWM device*/
    pwm_mode_t mode;    /**< PWM device mode */
    uint32_t freq;      /**< PWM device frequency */
    uint16_t res;       /**< PWM device resolution */

    umdk_pwm_ch_t pwm_chs[UMDK_PWM_NUM_CH_MAX];	/**< Configuration of PWM channels*/

    bool is_started;    /**< PWM device is running */
} umdk_pwm_dev_t;


/**
 * @brief UMDK-PWM module commands list
 */
typedef enum {
    UMDK_PWM_CMD_SET = 0, /**< Sets frequency and duty cycle for specified PWM channel  */
} umdk_pwm_cmd_t;

void umdk_pwm_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback);
bool umdk_pwm_cmd(module_data_t *data, module_data_t *reply);

#endif /* UMDK_PWM_H */
