/*
 * Copyright (c) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx127x
 * @{
 * @file
 * @brief       implementation of internal functions for sx127x
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "net/lora.h"

#include "sx127x.h"
#include "sx127x_registers.h"
#include "sx127x_internal.h"
#include "sx127x_params.h"

#include "lptimer.h"
#include "xtimer.h"

#define ENABLE_DEBUG (1)
#include "debug.h"


#define SX127X_SPI_SPEED    (SPI_CLK_1MHZ)
#define SX127X_SPI_MODE     (SPI_MODE_0)

int sx127x_check_version(sx127x_t *dev)
{
    /* Read version number and compare with sx127x assigned revision */
    uint8_t version = sx127x_reg_read(dev, SX127X_REG_VERSION);
    
    switch (version) {
        case VERSION_SX1272:
            dev->_internal.modem_chip = SX127X_MODEM_SX1272;
            puts("SX1272/73 transceiver detected");
            break;
        case VERSION_SX1276:
            dev->_internal.modem_chip = SX127X_MODEM_SX1276;
            puts("SX1276/77/78/79 transceiver detected");
            break;
        default:
            printf("[Error] sx127x test failed, invalid version number: %d\n",
                   version);
            return -1;
    }
    return 0;
}

void sx127x_reg_write(const sx127x_t *dev, uint8_t addr, uint8_t data)
{
    sx127x_reg_write_burst(dev, addr, &data, 1);
}

uint8_t sx127x_reg_read(const sx127x_t *dev, uint8_t addr)
{
    uint8_t data;

    sx127x_reg_read_burst(dev, addr, &data, 1);

    return data;
}

void sx127x_reg_write_burst(const sx127x_t *dev, uint8_t addr, uint8_t *buffer,
                            uint8_t size)
{
    spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX127X_SPI_MODE, SX127X_SPI_SPEED);

    gpio_clear(dev->params.nss_pin);
    spi_transfer_regs(dev->params.spi, SPI_CS_UNDEF, addr | 0x80, (char *) buffer, NULL, size);
    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi);
}

void sx127x_reg_read_burst(const sx127x_t *dev, uint8_t addr, uint8_t *buffer,
                           uint8_t size)
{
    spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX127X_SPI_MODE, SX127X_SPI_SPEED);

    gpio_clear(dev->params.nss_pin);
    spi_transfer_regs(dev->params.spi, SPI_CS_UNDEF, addr & 0x7F, NULL, (char *) buffer, size);
    gpio_set(dev->params.nss_pin);

    spi_release(dev->params.spi);
}

void sx127x_write_fifo(const sx127x_t *dev, uint8_t *buffer, uint8_t size)
{
    sx127x_reg_write_burst(dev, 0, buffer, size);
}

void sx127x_read_fifo(const sx127x_t *dev, uint8_t *buffer, uint8_t size)
{
    sx127x_reg_read_burst(dev, 0, buffer, size);
}

void sx1276_rx_chain_calibration(sx127x_t *dev)
{
    uint8_t reg_pa_config_init_val;
    uint32_t initial_freq;

    /* Save context */
    reg_pa_config_init_val = sx127x_reg_read(dev, SX127X_REG_PACONFIG);
    initial_freq = (double) (((uint32_t) sx127x_reg_read(dev, SX127X_REG_FRFMSB) << 16)
                             | ((uint32_t) sx127x_reg_read(dev, SX127X_REG_FRFMID) << 8)
                             | ((uint32_t) sx127x_reg_read(dev, SX127X_REG_FRFLSB))) * (double)LORA_FREQUENCY_RESOLUTION_DEFAULT;

    /* Cut the PA just in case, RFO output, power = -1 dBm */
    sx127x_reg_write(dev, SX127X_REG_PACONFIG, 0x00);

    /* Launch Rx chain calibration for LF band */
    sx127x_reg_write(dev,
                     SX127X_REG_IMAGECAL,
                     (sx127x_reg_read(dev, SX127X_REG_IMAGECAL) & SX127X_RF_IMAGECAL_IMAGECAL_MASK)
                     | SX127X_RF_IMAGECAL_IMAGECAL_START);

    while ((sx127x_reg_read(dev, SX127X_REG_IMAGECAL) & SX127X_RF_IMAGECAL_IMAGECAL_RUNNING)
           == SX127X_RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    /* Set a frequency in HF band */
    sx127x_set_channel(dev, SX127X_HF_CHANNEL_DEFAULT);

    /* Launch Rx chain calibration for HF band */
    sx127x_reg_write(dev,
                     SX127X_REG_IMAGECAL,
                     (sx127x_reg_read(dev, SX127X_REG_IMAGECAL) & SX127X_RF_IMAGECAL_IMAGECAL_MASK)
                     | SX127X_RF_IMAGECAL_IMAGECAL_START);
    while ((sx127x_reg_read(dev, SX127X_REG_IMAGECAL) & SX127X_RF_IMAGECAL_IMAGECAL_RUNNING)
           == SX127X_RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    /* Restore context */
    sx127x_reg_write(dev, SX127X_REG_PACONFIG, reg_pa_config_init_val);
    sx127x_set_channel(dev, initial_freq);
}

int16_t sx127x_read_rssi(const sx127x_t *dev)
{
    int16_t rssi = 0;

    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            rssi = -(sx127x_reg_read(dev, SX127X_REG_RSSIVALUE) >> 1);
            break;
        case SX127X_MODEM_LORA:
            if (dev->_internal.modem_chip == SX127X_MODEM_SX1272) {
                rssi = SX127X_RSSI_OFFSET + sx127x_reg_read(dev, SX127X_REG_LR_RSSIVALUE);
            } else {                
                if (dev->settings.channel > SX127X_RF_MID_BAND_THRESH) {
                    rssi = SX127X_RSSI_OFFSET_HF + sx127x_reg_read(dev, SX127X_REG_LR_RSSIVALUE);
                }
                else {
                    rssi = SX127X_RSSI_OFFSET_LF + sx127x_reg_read(dev, SX127X_REG_LR_RSSIVALUE);
                }
            }
            break;
        default:
            rssi = -1;
            break;
    }

    return rssi;
}

void sx127x_start_cad(sx127x_t *dev)
{
    switch (dev->settings.modem) {
        case SX127X_MODEM_FSK:
            break;
        case SX127X_MODEM_LORA:
            /* Disable all interrupts except CAD-related */
            sx127x_reg_write(dev, SX127X_REG_LR_IRQFLAGSMASK,
                             SX127X_RF_LORA_IRQFLAGS_RXTIMEOUT |
                             SX127X_RF_LORA_IRQFLAGS_RXDONE |
                             SX127X_RF_LORA_IRQFLAGS_PAYLOADCRCERROR |
                             SX127X_RF_LORA_IRQFLAGS_VALIDHEADER |
                             SX127X_RF_LORA_IRQFLAGS_TXDONE |
                             /*SX127X_RF_LORA_IRQFLAGS_CADDONE |*/
                             SX127X_RF_LORA_IRQFLAGS_FHSSCHANGEDCHANNEL
                             /* | SX127X_RF_LORA_IRQFLAGS_CADDETECTED*/
                             );

            // DIO3 = CADDone
            sx127x_reg_write(dev,
                             SX127X_REG_DIOMAPPING1,
                             (sx127x_reg_read(dev, SX127X_REG_DIOMAPPING1) &
                             SX127X_RF_LORA_DIOMAPPING1_DIO3_MASK) |
                             SX127X_RF_LORA_DIOMAPPING1_DIO3_00);
                             
            sx127x_set_state(dev,  SX127X_RF_CAD);
            sx127x_set_op_mode(dev, SX127X_RF_LORA_OPMODE_CAD);
            break;
        default:
            break;
    }
}

#if defined(LORAMAC_CHANNEL_FREE_DETECT_PREAMBLE)
typedef enum {
    SX127X_CAD_RUNNING,
    SX127X_CAD_CHANNEL_FREE,
    SX127X_CAD_CHANNEL_ACTIVE,
} sx127x_lora_cad_progress_t;

static sx127x_lora_cad_progress_t sx127x_cad_result;

void sx127x_lora_cad_done(bool activity) {
    if (activity) {
        sx127x_cad_result = SX127X_CAD_CHANNEL_ACTIVE;
    } else {
        sx127x_cad_result = SX127X_CAD_CHANNEL_FREE;
    }
}
#endif

bool sx127x_is_channel_free(sx127x_t *dev, uint32_t freq, int16_t rssi_threshold)
{
#if !defined(LORAMAC_CHANNEL_FREE_DETECT_PREAMBLE)
    /* using  RSSI value */
    int16_t rssi = 0;

    sx127x_set_channel(dev, freq);
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_RECEIVER);

    xtimer_spin(xtimer_ticks_from_usec(1000)); /* wait 1 millisecond */

    rssi = sx127x_read_rssi(dev);
    sx127x_set_sleep(dev);

    return (rssi <= rssi_threshold);
#else
    /* using  RSSI value */
    int16_t rssi = 0;

    sx127x_set_channel(dev, freq);
    sx127x_set_op_mode(dev, SX127X_RF_OPMODE_RECEIVER);

    xtimer_spin(xtimer_ticks_from_usec(1000)); /* wait 1 millisecond */

    rssi = sx127x_read_rssi(dev);
    sx127x_set_sleep(dev);

    if (rssi > rssi_threshold) {
        DEBUG("[sx127x] RSSI above threshold\n");
        return false;
    }

    /* using  LoRa CAD */
    int delay_ms = 5 + ((100 + 10*dev->settings.lora.datarate) >> dev->settings.lora.datarate);
    
    for (int k = 0; k < 10; k++) {
        sx127x_cad_result = SX127X_CAD_RUNNING;

        sx127x_start_cad(dev);
        lptimer_sleep(delay_ms);
        sx127x_set_sleep(dev);

        if (sx127x_cad_result == SX127X_CAD_CHANNEL_ACTIVE) {
            DEBUG("[sx127x] channel activity detected\n");
            return false;

            /* otherwise restart CAD after a pause */
            lptimer_sleep(100);
        }
    }

    DEBUG("[sx127x] channel is free\n");

    return true;
#endif
}
