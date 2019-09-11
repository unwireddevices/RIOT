/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
 
/**
 * @ingroup     boards_unwd-railwaycar
 * @{
 *
 * @file
 * @brief       Board configuration for the unwd-railwaycar (nRF52832)
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 * @author      Oleg Artamonov
 *
 */

#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LED pin configuration
 * @{
 */

#define LED_PORT            (NRF_P0)
#define LED0_MASK           (1 << 24)
#define LED0_ON             (LED_PORT->OUTCLR = LED0_MASK)
#define LED0_OFF            (LED_PORT->OUTSET = LED0_MASK)
#define LED0_TOGGLE         (LED_PORT->OUT   ^= LED0_MASK)

/** @} */

#define RWCAR_BAT_VCC       GPIO_PIN(0, 2)
#define RWCAR_DC_SLEEP      GPIO_PIN(0, 3)
#define RWCAR_ACC_IRQ2      GPIO_PIN(0, 4)
#define RWCAR_ACC_IRQ1      GPIO_PIN(0, 5)
#define RWCAR_I2C_SCL       GPIO_PIN(0, 6)
#define RWCAR_I2C_SDA       GPIO_PIN(0, 7)
#define RWCAR_GPS_RX        GPIO_PIN(0, 12)
#define RWCAR_GPS_TX        GPIO_PIN(0, 13)
#define RWCAR_GPS_BACKUP    GPIO_PIN(0, 14)
#define RWCAR_GPS_POWER     GPIO_PIN(0, 15)
#define RWCAR_GPS_ENABLE    GPIO_PIN(0, 16)
#define RWCAR_GSM_POWER     GPIO_PIN(0, 17)
#define RWCAR_GSM_ENABLE    GPIO_PIN(0, 18)
#define RWCAR_GSM_RX        GPIO_PIN(0, 19)
#define RWCAR_GSM_TX        GPIO_PIN(0, 20)
#define RWCAR_SHELL_TX      GPIO_PIN(0, 22)
#define RWCAR_SHELL_RX      GPIO_PIN(0, 23)

#define RWCAR_PARAM_I2C     I2C_DEV(0)

/**
 * @brief   Initialize the platform
 */
void board_init(void);


#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
