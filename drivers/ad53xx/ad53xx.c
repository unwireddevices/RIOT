/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     drivers_ad53xx
 * @{
 *
 * @file
 * @brief       Driver for the Analog Devices AD5308/AD5318/AD5328 DACs.
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include <stdint.h>
#include <stdbool.h>
#include "xtimer.h"
#include "ad53xx.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define AD53XX_COMMAND      (1<<15)

#define AD53XX_LDAC_LOW     (1<<13)
#define AD53XX_LDAC_HIGH    (1<<13 | 1)
#define AD53XX_LDAC_UPDATE  (1<<13 | 2)

#define AD53XX_CHANNEL_PD   (1<<14)

#define AD53XX_RESET_DATA   (1<<14 | 1<<13)
#define AD53XX_RESET_ALL    (1<<14 | 1<<13 | 1 << 12)

#define AD53XX_GAIN_2VREF   (1<<5 | 1<<4)
#define AD53XX_BUF_REF      (1<<3 | 1<<2)
#define AD53XX_REF_VDD      (1<<1 | 1)

/**
 * @brief Write a register value to the ADC
 *
 * @param[in]  dev    device descriptor
 * @param[in]  addr   channel address
 * @param[in]  buf    source buffer
 *
 * @return            0 on success
 * @return            -1 on communication errors
 */
static int ad53xx_write_reg(const ad53xx_t *dev, uint16_t data)
{
    int status = 0;
    
    /* Acquire exclusive access to the bus. */
    spi_acquire(dev->spi);
    /* Perform the transaction */
    gpio_clear(dev->cs);
    
    uint16_t data_in;
    
    spi_transfer_bytes(dev->spi, (char *)&data, (char *)&data_in, 2);

    gpio_set(dev->cs);
    /* Release the bus for other threads. */
    spi_release(dev->spi);
    
    return status;
}

int ad53xx_init(ad53xx_t *dev, spi_t spi, gpio_t cs, gpio_t ldac)
{
    /* write device descriptor */
    dev->spi = spi;
    dev->cs = cs;
    dev->ldac = ldac;
    
    dev->initialized = false;

    /* CS */
    gpio_init(dev->cs, GPIO_OUT);
    gpio_set(dev->cs);
    
    /* LDAC */
    gpio_init(dev->ldac, GPIO_OUT);
    gpio_set(dev->ldac);
    
    uint16_t data = AD53XX_COMMAND | AD53XX_RESET_ALL;
    ad53xx_write_reg(dev, data);
    
    data = AD53XX_COMMAND | AD53XX_LDAC_UPDATE;
    ad53xx_write_reg(dev, data);

    dev->initialized = true;
    return 0;
}

int ad53xx_set_single(ad53xx_t *dev, uint8_t channel, uint16_t data)
{
    assert(channel < 8);
    
    uint8_t data_shift = 0;
#ifdef AD53XX_AD5308
    data_shift = 4;
#endif
#ifdef AD53XX_AD5318
    data_shift = 2;
#endif
    
    uint16_t cmd = (channel << 12) | (data << data_shift);
    ad53xx_write_reg(dev, cmd);
    
    // update DAC outputs
    gpio_clear(dev->ldac);
    xtimer_spin(xtimer_ticks_from_usec(1000));
    gpio_set(dev->ldac);
    
    return 0;
}

int ad53xx_set_all(ad53xx_t *dev, uint16_t *data)
{
    uint8_t data_shift = 0;
#ifdef AD53XX_AD5308
    data_shift = 4;
#endif
#ifdef AD53XX_AD5318
    data_shift = 2;
#endif

    int i = 0;
    for (i = 0; i < 8; i++) {
        uint16_t cmd = (i << 12) | (data[i] << data_shift);
        ad53xx_write_reg(dev, cmd);
    }
    
    // update DAC outputs
    gpio_clear(dev->ldac);
    xtimer_spin(xtimer_ticks_from_usec(1000));
    gpio_set(dev->ldac);
    
    return 0;
}

