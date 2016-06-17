/*
 * Copyright (C) 2016 cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx1276
 * @{
 * @file
 * @brief       Basic functionality of sx1276 driver
 *
 * @author      Cr0s
 * @}
 */
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "periph/gpio.h"
#include "periph/spi.h"

#include "xtimer.h"
#include "thread.h"

#include "sx1276.h"
#include "include/sx1276_regs_fsk.h"
#include "include/sx1276_regs_lora.h"

static char stack[THREAD_STACKSIZE_MAIN];
static msg_t msg_queue[10];

/**
 * These functions must be implemented in user's code
 */
void sx1276_board_set_ant_sw_low_power(uint8_t lp);
void sx1276_board_set_ant_sw(uint8_t tx);

/**
 * Radio registers definition
 */
typedef struct {
    sx1276_radio_modems_t modem;
    uint8_t addr;
    uint8_t value;

} sx1276_radio_registers_t;

/*
 * Private functions prototypes
 */

/**
 * @brief Performs the Rx chain calibration for LF and HF bands
 * Must be called just after the reset so all registers are at their
 *         default values
 */
static void _rx_chain_calibration(sx1276_t *dev);

/**
 * @brief Resets the SX1276
 */
void sx1276_reset(sx1276_t *dev);

/**
 * @brief Sets the SX1276 in transmission mode for the given time
 * @param [IN] timeout Transmission timeout [us] [0: continuous, others timeout]
 */
void sx1276_set_tx(sx1276_t *dev, uint32_t timeout);

/**
 * @brief Writes the buffer contents to the SX1276 FIFO
 *
 * @param [IN] buffer Buffer containing data to be put on the FIFO.
 * @param [IN] size Number of bytes to be written to the FIFO
 */
void sx1276_write_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Reads the contents of the SX1276 FIFO
 *
 * @param [OUT] buffer Buffer where to copy the FIFO read data.
 * @param [IN] size Number of bytes to be read from the FIFO
 */
void sx1276_read_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Sets the SX1276 operating mode
 *
 * @param [IN] op_mode New operating mode
 */
void sx1276_set_op_mode(sx1276_t *dev, uint8_t op_mode);

/*
 * SX1276 DIO _irq callback functions prototype
 */

/**
 * @brief DIO 0 _irq callback
 */
void sx1276_on_dio_0_isr(sx1276_t *dev);

/**
 * @brief DIO 1 _irq callback
 */
void sx1276_on_dio_1_isr(sx1276_t *dev);

/**
 * @brief DIO 2 _irq callback
 */
void sx1276_on_dio_2_isr(sx1276_t *dev);

/**
 * @brief DIO 3 _irq callback
 */
void sx1276_on_dio_3_isr(sx1276_t *dev);

/**
 * @brief DIO 4 _irq callback
 */
void sx1276_on_dio_4_isr(sx1276_t *dev);

/**
 * @brief DIO 5 _irq callback
 */
void sx1276_on_dio_5_isr(sx1276_t *dev);

/**
 * @brief Tx & Rx timeout timer callback
 */
void sx1276_on_timeout_isr(sx1276_t *dev);

/*
 * Private global constants
 */

/**
 * Constant values need to compute the RSSI value
 */
#define RSSI_OFFSET_LF                              -164
#define RSSI_OFFSET_HF                              -157

/*
 * Private global variables
 */

/**
 * Tx and Rx timers
 */
// TODO: use of RIOT timers
/*TimerEvent_t TxTimeoutTimer;
   TimerEvent_t RxTimeoutTimer;
   TimerEvent_t RxTimeoutSyncWord;*/

static void sx1276_set_status(sx1276_t *dev, sx1276_radio_state_t state)
{
    dev->settings.state = state;
}

/**
 * @brief SX1276 DIO interrupt handlers initialization
 */

static void _init_isrs(sx1276_t *dev)
{
    gpio_init_int(dev->dio0_pin, GPIO_IN, GPIO_RISING, sx1276_on_dio0_isr, dev);
    gpio_init_int(dev->dio1_pin, GPIO_IN, GPIO_RISING, sx1276_on_dio1_isr, dev);
    gpio_init_int(dev->dio2_pin, GPIO_IN, GPIO_RISING, sx1276_on_dio2_isr, dev);
    gpio_init_int(dev->dio3_pin, GPIO_IN, GPIO_RISING, sx1276_on_dio3_isr, dev);
}

void sx1276_init(sx1276_t *dev)
{
    // Initialize driver timeout timers
    // TODO: use of RIOT timers
    /*TimerInit( &TxTimeoutTimer, SX1276OnTimeoutIrq );
       TimerInit( &RxTimeoutTimer, SX1276OnTimeoutIrq );
       TimerInit( &RxTimeoutSyncWord, SX1276OnTimeoutIrq );*/

    sx1276_reset(dev);

    _init_isrs(dev);
    _rx_chain_calibration(dev);

    sx1276_reg_write(dev, REG_OPMODE, 0x00); /* Set RegOpMode value to the datasheet's default */
    sx1276_set_modem(dev, MODEM_LORA);

    sx1276_set_channel(dev, dev->settings.channel);

    kernel_pid_t pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN,
                                     THREAD_CREATE_STACKTEST, dio_polling_thread, dev,
                                     "sx1276_dio_polling_thread");

    if (pid <= KERNEL_PID_UNDEF) {
        puts("sx1276: creation of DIO polling thread failed");
        return; // TODO: error codes
    }

    dev->dio_polling_thread_pid = pid;
}

sx1276_radio_state_t sx1276_get_status(sx1276_t *dev)
{
    return dev->settings.state;
}

void sx1276_set_channel(sx1276_t *dev, uint32_t freq)
{
    /* Save current operating mode */
    uint8_t prev_mode = sx1276_reg_read(dev, REG_OPMODE);

    sx1276_set_op_mode(dev, RF_OPMODE_STANDBY);
    dev->settings.channel = freq;

    freq = (uint32_t)((double) freq / (double) FREQ_STEP);

    /* Write frequency settings into chip */
    sx1276_reg_write(dev, REG_FRFMSB, (uint8_t)((freq >> 16) & 0xFF));
    sx1276_reg_write(dev, REG_FRFMID, (uint8_t)((freq >> 8) & 0xFF));
    sx1276_reg_write(dev, REG_FRFLSB, (uint8_t)(freq & 0xFF));

    /* Restore previous operating mode */
    sx1276_reg_write(dev, REG_OPMODE, prev_mode);
}

bool sx1276_test(sx1276_t *dev)
{
    /* Read version number and compare with sx1276 assigned revision */
    uint8_t version = sx1276_reg_read(dev, REG_VERSION);

    if (version != VERSION_SX1276 || version == 0x1C) {
        printf("sx1276: test failed, invalid version number: %d\n", version);
        return false;
    }

    return true;
}

bool sx1276_is_channel_free(sx1276_t *dev, uint32_t freq, uint16_t rssi_thresh)
{
    int16_t rssi = 0;

    sx1276_set_channel(dev, freq);
    sx1276_set_op_mode(dev, RF_OPMODE_RECEIVER);

    xtimer_usleep(1000); /* wait 1 millisecond */

    rssi = sx1276_read_rssi(dev);
    sx1276_set_sleep(dev);

    if (rssi > rssi_thresh) {
        return false;
    }

    return true;
}

void sx1276_set_modem(sx1276_t *dev, sx1276_radio_modems_t modem)
{
    dev->settings.modem = modem;

    switch (dev->settings.modem) {
        case MODEM_LORA:
            sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);
            sx1276_reg_write(dev,
                             REG_OPMODE,
                             (sx1276_reg_read(dev, REG_OPMODE)
                              & RFLR_OPMODE_LONGRANGEMODE_MASK)
                             | RFLR_OPMODE_LONGRANGEMODE_ON);

            sx1276_reg_write(dev, REG_DIOMAPPING1, 0x00);
            sx1276_reg_write(dev, REG_DIOMAPPING2, 0x10); /* DIO5=ClkOut */
            break;

        case MODEM_FSK:
            sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);
            sx1276_reg_write(dev,
                             REG_OPMODE,
                             (sx1276_reg_read(dev, REG_OPMODE)
                              & RFLR_OPMODE_LONGRANGEMODE_MASK)
                             | RFLR_OPMODE_LONGRANGEMODE_OFF);

            sx1276_reg_write(dev, REG_DIOMAPPING1, 0x00);
            //sx1276_reg_write(dev, REG_DIOMAPPING2, 0x20); /* DIO5=mode_ready */
            break;
        default:
            break;
    }
}

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70

uint32_t sx1276_random(sx1276_t *dev)
{
    uint8_t i;
    uint32_t rnd = 0;

    sx1276_set_modem(dev, MODEM_LORA); /* Set LoRa modem ON */

    /* Disable LoRa modem interrupts */
    sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                     RFLR_IRQFLAGS_RXDONE |
                     RFLR_IRQFLAGS_PAYLOADCRCERROR |
                     RFLR_IRQFLAGS_VALIDHEADER |
                     RFLR_IRQFLAGS_TXDONE |
                     RFLR_IRQFLAGS_CADDONE |
                     RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                     RFLR_IRQFLAGS_CADDETECTED);

    sx1276_set_op_mode(dev, RF_OPMODE_STANDBY);
    sx1276_reg_write(dev, REG_LR_MODEMCONFIG1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1);
    sx1276_reg_write(dev, REG_LR_MODEMCONFIG1, RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2);

    /* Set radio in continuous reception */
    sx1276_set_op_mode(dev, RF_OPMODE_RECEIVER);

    for (i = 0; i < 32; i++) {
        xtimer_usleep(1000); /* wait for the chaos */

        /* Non-filtered RSSI value reading. Only takes the LSB value */
        rnd |= ((uint32_t) sx1276_reg_read(dev, REG_LR_RSSIWIDEBAND) & 0x01) << i;
    }

    sx1276_set_sleep(dev);

    return rnd;
}

/**
 * @brief Performs the Rx chain calibration for LF and HF bands
 * @note Must be called just after the reset so all registers are at their
 *         default values
 */
static void _rx_chain_calibration(sx1276_t *dev)
{
    uint8_t reg_pa_config_init_val;
    uint32_t initial_freq;

    /* Save context */
    reg_pa_config_init_val = sx1276_reg_read(dev, REG_PACONFIG);
    initial_freq = (double) (((uint32_t) sx1276_reg_read(dev, REG_FRFMSB) << 16)
                             | ((uint32_t) sx1276_reg_read(dev, REG_FRFMID) << 8)
                             | ((uint32_t) sx1276_reg_read(dev, REG_FRFLSB))) * (double) FREQ_STEP;

    /* Cut the PA just in case, RFO output, power = -1 dBm */
    sx1276_reg_write(dev, REG_PACONFIG, 0x00);

    /* Launch Rx chain calibration for LF band */
    sx1276_reg_write(dev,
                     REG_IMAGECAL,
                     (sx1276_reg_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK)
                     | RF_IMAGECAL_IMAGECAL_START);

    while ((sx1276_reg_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
           == RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    /* Set a frequency in HF band */
    sx1276_set_channel(dev, CHANNEL_HF);

    /* Launch Rx chain calibration for HF band */
    sx1276_reg_write(dev,
                     REG_IMAGECAL,
                     (sx1276_reg_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_MASK)
                     | RF_IMAGECAL_IMAGECAL_START);
    while ((sx1276_reg_read(dev, REG_IMAGECAL) & RF_IMAGECAL_IMAGECAL_RUNNING)
           == RF_IMAGECAL_IMAGECAL_RUNNING) {
    }

    /* Restore context */
    sx1276_reg_write(dev, REG_PACONFIG, reg_pa_config_init_val);
    sx1276_set_channel(dev, initial_freq);
}

void sx1276_set_rx_config(sx1276_t *dev, sx1276_radio_modems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint32_t bandwidth_afc, uint16_t preamble_len,
                          uint16_t symb_timeout, bool implicit_header,
                          uint8_t payload_len,
                          bool crc_on, bool freq_hop_on, uint8_t hop_period,
                          bool iq_inverted, bool rx_continuous)
{
    sx1276_set_modem(dev, modem);

    switch (modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        {
            if (bandwidth > 2) {
                /* Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported */
                /* TODO: error codes */
                while (1) {
                }
            }

            bandwidth += 7;

            dev->settings.lora.bandwidth = bandwidth;
            dev->settings.lora.datarate = datarate;
            dev->settings.lora.coderate = coderate;
            dev->settings.lora.preamble_len = preamble_len;
            dev->settings.lora.implicit_header = implicit_header;
            dev->settings.lora.payload_len = payload_len;
            dev->settings.lora.crc_on = crc_on;
            dev->settings.lora.freq_hop_on = freq_hop_on;
            dev->settings.lora.hop_period = hop_period;
            dev->settings.lora.iq_inverted = iq_inverted;
            dev->settings.lora.rx_continuous = rx_continuous;

            if (datarate > 12) {
                datarate = 12;
            }
            else if (datarate < 6) {
                datarate = 6;
            }

            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12)))
                || ((bandwidth == 8) && (datarate == 12))) {
                dev->settings.lora.low_datarate_optimize = 0x01;
            }
            else {
                dev->settings.lora.low_datarate_optimize = 0x00;
            }

            sx1276_reg_write(dev,
                             REG_LR_MODEMCONFIG1,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG1) &
                              RFLR_MODEMCONFIG1_BW_MASK &
                              RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                              RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (bandwidth << 4)
                             | (coderate << 1) | implicit_header);

            sx1276_reg_write(dev, REG_LR_MODEMCONFIG2,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG2) &
                              RFLR_MODEMCONFIG2_SF_MASK &
                              RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
                              RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK) | (datarate << 4)
                             | (crc_on << 2)
                             | ((symb_timeout >> 8)
                                & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK));

            sx1276_reg_write(dev,
                             REG_LR_MODEMCONFIG3,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG3)
                              & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK)
                             | (dev->settings.lora.low_datarate_optimize << 3));

            sx1276_reg_write(dev, REG_LR_SYMBTIMEOUTLSB,
                             (uint8_t)(symb_timeout & 0xFF));

            sx1276_reg_write(dev, REG_LR_PREAMBLEMSB,
                             (uint8_t)((preamble_len >> 8) & 0xFF));
            sx1276_reg_write(dev, REG_LR_PREAMBLELSB,
                             (uint8_t)(preamble_len & 0xFF));

            if (!implicit_header) {
                sx1276_reg_write(dev, REG_LR_PAYLOADLENGTH, payload_len);
            }


            if (dev->settings.lora.freq_hop_on) {
                sx1276_reg_write(dev,
                                 REG_LR_PLLHOP,
                                 (sx1276_reg_read(dev, REG_LR_PLLHOP)
                                  & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                sx1276_reg_write(dev, REG_LR_HOPPERIOD,
                                 dev->settings.lora.hop_period);
            }

            if ((bandwidth == 9) && (RF_MID_BAND_THRESH)) {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x02);
                sx1276_reg_write(dev, REG_LR_TEST3A, 0x64);
            }
            else if (bandwidth == 9) {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x02);
                sx1276_reg_write(dev, REG_LR_TEST3A, 0x7F);
            }
            else {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x03);
            }

            if (datarate == 6) {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE,
                                 (sx1276_reg_read(dev, REG_LR_DETECTOPTIMIZE) &
                                  RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF6);
                sx1276_reg_write(dev, REG_LR_DETECTIONTHRESHOLD,
                                 RFLR_DETECTIONTHRESH_SF6);
            }
            else {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                sx1276_reg_write(dev, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

uint8_t sx1276_get_pa_select( uint32_t channel )
{
    if (channel < RF_MID_BAND_THRESH) {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else {
        return RF_PACONFIG_PASELECT_RFO;
    }
}


void sx1276_set_tx_config(sx1276_t *dev, sx1276_radio_modems_t modem, int8_t power, uint32_t fdev,
                          uint32_t bandwidth, uint32_t datarate,
                          uint8_t coderate, uint16_t preamble_len,
                          bool implicit_header, bool crc_on, bool freq_hop_on,
                          uint8_t hop_period, bool iq_inverted, uint32_t timeout)
{
    uint8_t pa_config = 0;
    uint8_t pa_dac = 0;

    sx1276_set_modem(dev, modem);

    pa_config = sx1276_reg_read(dev, REG_PACONFIG);
    pa_dac = sx1276_reg_read(dev, REG_PADAC);

    pa_config = (pa_config & RF_PACONFIG_PASELECT_MASK) | sx1276_get_pa_select(dev->settings.channel) << 7;
    pa_config = (pa_config & RF_PACONFIG_MAX_POWER_MASK) | (0x05 << 4); // max power is 14dBm

    sx1276_reg_write(dev, REG_PARAMP, RF_PARAMP_0050_US);

    if ((pa_config & RF_PACONFIG_PASELECT_PABOOST)
        == RF_PACONFIG_PASELECT_PABOOST) {
        if (power > 17) {
            pa_dac = (pa_dac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else {
            pa_dac = (pa_dac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((pa_dac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON) {
            if (power < 5) {
                power = 5;
            }
            if (power > 20) {
                power = 20;
            }
            pa_config = (pa_config & RF_PACONFIG_OUTPUTPOWER_MASK)
                        | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else {
            if (power < 2) {
                power = 2;
            }
            if (power > 17) {
                power = 17;
            }

            pa_config = (pa_config & RF_PACONFIG_OUTPUTPOWER_MASK)
                        | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else {
        if (power < -1) {
            power = -1;
        }
        if (power > 14) {
            power = 14;
        }

        pa_config = (pa_config & RF_PACONFIG_OUTPUTPOWER_MASK)
                    | (uint8_t)((uint16_t)(power + 1) & 0x0F);
    }

    sx1276_reg_write(dev, REG_PACONFIG, pa_config);
    sx1276_reg_write(dev, REG_PADAC, pa_dac);

    switch (modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        {
            if (bandwidth > 2) {
                /* Fatal error: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported */
                /* TODO: error codes */
                while (1) {
                }
            }

            bandwidth += 7;

            dev->settings.lora.bandwidth = bandwidth;
            dev->settings.lora.datarate = datarate;
            dev->settings.lora.coderate = coderate;
            dev->settings.lora.preamble_len = preamble_len;
            dev->settings.lora.implicit_header = implicit_header;
            dev->settings.lora.crc_on = crc_on;
            dev->settings.lora.freq_hop_on = freq_hop_on;
            dev->settings.lora.hop_period = hop_period;
            dev->settings.lora.iq_inverted = iq_inverted;

            if (datarate > 12) {
                datarate = 12;
            }
            else if (datarate < 6) {
                datarate = 6;
            }

            if (((bandwidth == 7) && ((datarate == 11) || (datarate == 12)))
                || ((bandwidth == 8) && (datarate == 12))) {
                dev->settings.lora.low_datarate_optimize = 0x01;
            }
            else {
                dev->settings.lora.low_datarate_optimize = 0x00;
            }

            sx1276_reg_write(dev, REG_LR_MODEMCONFIG1,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG1) &
                              RFLR_MODEMCONFIG1_BW_MASK &
                              RFLR_MODEMCONFIG1_CODINGRATE_MASK &
                              RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK) | (bandwidth << 4)
                             | (coderate << 1) | implicit_header);

            sx1276_reg_write(dev, REG_LR_MODEMCONFIG2,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG2) &
                              RFLR_MODEMCONFIG2_SF_MASK &
                              RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK) | (datarate << 4)
                             | (crc_on << 2));

            sx1276_reg_write(dev,
                             REG_LR_MODEMCONFIG3,
                             (sx1276_reg_read(dev, REG_LR_MODEMCONFIG3)
                              & RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK)
                             | (dev->settings.lora.low_datarate_optimize << 3));

            sx1276_reg_write(dev, REG_LR_PREAMBLEMSB,
                             (uint8_t)((preamble_len >> 8) & 0xFF));
            sx1276_reg_write(dev, REG_LR_PREAMBLELSB,
                             (uint8_t)(preamble_len & 0xFF));


            if (dev->settings.lora.freq_hop_on) {
                sx1276_reg_write(dev,
                                 REG_LR_PLLHOP,
                                 (sx1276_reg_read(dev, REG_LR_PLLHOP)
                                  & RFLR_PLLHOP_FASTHOP_MASK) | RFLR_PLLHOP_FASTHOP_ON);
                sx1276_reg_write(dev, REG_LR_HOPPERIOD,
                                 dev->settings.lora.hop_period);
            }

            if ((bandwidth == 9) && (RF_MID_BAND_THRESH)) {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x02);
                sx1276_reg_write(dev, REG_LR_TEST3A, 0x64);
            }
            else if (bandwidth == 9) {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x02);
                sx1276_reg_write(dev, REG_LR_TEST3A, 0x7F);
            }
            else {
                /* ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth */
                sx1276_reg_write(dev, REG_LR_TEST36, 0x03);
            }

            if (datarate == 6) {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE,
                                 (sx1276_reg_read(dev, REG_LR_DETECTOPTIMIZE) &
                                  RFLR_DETECTIONOPTIMIZE_MASK) |
                                 RFLR_DETECTIONOPTIMIZE_SF6);
                sx1276_reg_write(dev, REG_LR_DETECTIONTHRESHOLD,
                                 RFLR_DETECTIONTHRESH_SF6);
            }
            else {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE, RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12);
                sx1276_reg_write(dev, REG_LR_DETECTIONTHRESHOLD, RFLR_DETECTIONTHRESH_SF7_TO_SF12);
            }
        }
        break;
    }
}

uint32_t sx1276_get_time_on_air(sx1276_t *dev, sx1276_radio_modems_t modem,
                                uint8_t pkt_len)
{
    uint32_t air_time = 0;

    switch (modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        {
            double bw = 0.0;

            /* Note: When using LoRa modem only bandwidths 125, 250 and 500 kHz are supported. */
            switch (dev->settings.lora.bandwidth) {
                case 7: /* 125 kHz */
                    bw = 125e3;
                    break;
                case 8: /* 250 kHz */
                    bw = 250e3;
                    break;
                case 9: /* 500 kHz */
                    bw = 500e3;
                    break;
            }

            /* Symbol rate : time for one symbol [secs] */
            double rs = bw / (1 << dev->settings.lora.datarate);
            double ts = 1 / rs;

            /* time of preamble */
            double t_preamble = (dev->settings.lora.preamble_len + 4.25) * ts;

            /* Symbol length of payload and time */
            double tmp =
                ceil(
                    (8 * pkt_len - 4 * dev->settings.lora.datarate + 28
                     + 16 * dev->settings.lora.crc_on
                     - (!dev->settings.lora.implicit_header ? 20 : 0))
                    / (double) (4 * dev->settings.lora.datarate
                                - ((dev->settings.lora.low_datarate_optimize
                                    > 0) ? 2 : 0)))
                * (dev->settings.lora.coderate + 4);
            double n_payload = 8 + ((tmp > 0) ? tmp : 0);
            double t_payload = n_payload * ts;

            /* Time on air */
            double t_on_air = t_preamble + t_payload;

            /* return seconds */
            air_time = floor(t_on_air * 1e6 + 0.999);
        }
        break;
    }

    return air_time;
}

void sx1276_send(sx1276_t *dev, uint8_t *buffer, uint8_t size)
{
    switch (dev->settings.modem) {
        case MODEM_FSK:
            sx1276_write_fifo(dev, &size, 1);
            sx1276_write_fifo(dev, buffer, size);
            break;

        case MODEM_LORA:
        {

            if (dev->settings.lora.iq_inverted) {
                sx1276_reg_write(dev,
                                 REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, REG_LR_INVERTIQ)
                                   & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                  | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON));
                sx1276_reg_write(dev, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else {
                sx1276_reg_write(dev,
                                 REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, REG_LR_INVERTIQ)
                                   & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                  | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                sx1276_reg_write(dev, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            /* Initializes the payload size */
            sx1276_reg_write(dev, REG_LR_PAYLOADLENGTH, size);

            /* Full buffer used for Tx */
            sx1276_reg_write(dev, REG_LR_FIFOTXBASEADDR, 0x80);
            sx1276_reg_write(dev, REG_LR_FIFOADDRPTR, 0x80);

            /* FIFO operations can not take place in Sleep mode
             * So wake up the chip */
            if ((sx1276_reg_read(dev, REG_OPMODE) & ~RF_OPMODE_MASK)
                == RF_OPMODE_SLEEP) {
                puts("sx1276: waking up chip"); // XXX: debug

                sx1276_set_standby(dev);
                xtimer_usleep(RADIO_WAKEUP_TIME); /* wait for chip wake up */
            }

            /* Write payload buffer */
            sx1276_write_fifo(dev, buffer, size);
        }
        break;
    }

    /* Mask all interrupts except TXDONE */
    sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK,
                     RFLR_IRQFLAGS_RXTIMEOUT |
                     RFLR_IRQFLAGS_RXDONE |
                     RFLR_IRQFLAGS_PAYLOADCRCERROR |
                     RFLR_IRQFLAGS_VALIDHEADER |
                     //RFLR_IRQFLAGS_TXDONE |
                     RFLR_IRQFLAGS_CADDONE |
                     RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                     RFLR_IRQFLAGS_CADDETECTED);

    /* Max TXDONE interrupt to the DIO0 line */
    sx1276_reg_write(dev,
                     REG_DIOMAPPING1,
                     (sx1276_reg_read(dev, REG_DIOMAPPING1)
                      & RFLR_DIOMAPPING1_DIO0_MASK)
                     | RFLR_DIOMAPPING1_DIO0_01);

    /* Put chip into transfer mode */
    sx1276_set_status(dev,  RF_TX_RUNNING);
    //TimerStart(&TxTimeoutTimer); // TODO: use RIOT timers

    sx1276_set_op_mode(dev, RF_OPMODE_TRANSMITTER);
}

void sx1276_set_sleep(sx1276_t *dev)
{
    // TODO: use RIOT timers
    /*TimerStop( &RxTimeoutTimer );
       TimerStop( &TxTimeoutTimer );*/

    sx1276_set_op_mode(dev, RF_OPMODE_SLEEP);
    sx1276_set_status(dev,  RF_IDLE);
}

void sx1276_set_standby(sx1276_t *dev)
{
    // TODO: use RIOT timers
    /*TimerStop( &RxTimeoutTimer );
       TimerStop( &TxTimeoutTimer );*/

    sx1276_set_op_mode(dev, RF_OPMODE_STANDBY);
    sx1276_set_status(dev,  RF_IDLE);
}

void sx1276_set_rx(sx1276_t *dev, uint32_t timeout)
{
    bool rx_continuous = false;

    switch (dev->settings.modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        {
            if (dev->settings.lora.iq_inverted) {
                sx1276_reg_write(dev,
                                 REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, REG_LR_INVERTIQ)
                                   & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                  | RFLR_INVERTIQ_RX_ON | RFLR_INVERTIQ_TX_OFF));
                sx1276_reg_write(dev, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON);
            }
            else {
                sx1276_reg_write(dev,
                                 REG_LR_INVERTIQ,
                                 ((sx1276_reg_read(dev, REG_LR_INVERTIQ)
                                   & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK)
                                  | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF));
                sx1276_reg_write(dev, REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF);
            }

            /* ERRATA 2.3 - Receiver Spurious Reception of a LoRa Signal */
            if (dev->settings.lora.bandwidth < 9) {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE,
                                 sx1276_reg_read(dev, REG_LR_DETECTOPTIMIZE) & 0x7F);
                sx1276_reg_write(dev, REG_LR_TEST30, 0x00);
                switch (dev->settings.lora.bandwidth) {
                    case 0: // 7.8 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x48);
                        sx1276_set_channel(dev, dev->settings.channel + 7.81e3);
                        break;
                    case 1: // 10.4 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x44);
                        sx1276_set_channel(dev, dev->settings.channel + 10.42e3);
                        break;
                    case 2: // 15.6 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x44);
                        sx1276_set_channel(dev, dev->settings.channel + 15.62e3);
                        break;
                    case 3: // 20.8 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x44);
                        sx1276_set_channel(dev, dev->settings.channel + 20.83e3);
                        break;
                    case 4: // 31.2 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x44);
                        sx1276_set_channel(dev, dev->settings.channel + 31.25e3);
                        break;
                    case 5: // 41.4 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x44);
                        sx1276_set_channel(dev, dev->settings.channel + 41.67e3);
                        break;
                    case 6: // 62.5 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x40);
                        break;
                    case 7: // 125 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x40);
                        break;
                    case 8: // 250 kHz
                        sx1276_reg_write(dev, REG_LR_TEST2F, 0x40);
                        break;
                }
            }
            else {
                sx1276_reg_write(dev, REG_LR_DETECTOPTIMIZE,
                                 sx1276_reg_read(dev, REG_LR_DETECTOPTIMIZE) | 0x80);
            }

            rx_continuous = dev->settings.lora.rx_continuous;

            /* Setup interrupts */
            if (dev->settings.lora.freq_hop_on) {
                sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK,  //RFLR_IRQFLAGS_RXTIMEOUT |
                                                            //RFLR_IRQFLAGS_RXDONE |
                                                            //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                 RFLR_IRQFLAGS_VALIDHEADER |
                                 RFLR_IRQFLAGS_TXDONE |
                                 RFLR_IRQFLAGS_CADDONE |
                                 //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone, DIO2=FhssChangeChannel
                sx1276_reg_write(dev,
                                 REG_DIOMAPPING1,
                                 (sx1276_reg_read(dev, REG_DIOMAPPING1)
                                  & RFLR_DIOMAPPING1_DIO0_MASK
                                  & RFLR_DIOMAPPING1_DIO2_MASK)
                                 | RFLR_DIOMAPPING1_DIO0_00
                                 | RFLR_DIOMAPPING1_DIO2_00);
            }
            else {
                sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK,  //RFLR_IRQFLAGS_RXTIMEOUT |
                                                            //RFLR_IRQFLAGS_RXDONE |
                                                            //RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                 RFLR_IRQFLAGS_VALIDHEADER |
                                 RFLR_IRQFLAGS_TXDONE |
                                 RFLR_IRQFLAGS_CADDONE |
                                 RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=RxDone
                sx1276_reg_write(dev,
                                 REG_DIOMAPPING1,
                                 (sx1276_reg_read(dev, REG_DIOMAPPING1)
                                  & RFLR_DIOMAPPING1_DIO0_MASK)
                                 | RFLR_DIOMAPPING1_DIO0_00);
            }

            sx1276_reg_write(dev, REG_LR_FIFORXBASEADDR, 0);
            sx1276_reg_write(dev, REG_LR_FIFOADDRPTR, 0);
        }
        break;
    }

    sx1276_set_status(dev,  RF_RX_RUNNING);
    if (timeout != 0) {
        // TODO: use RIOT timers
        //TimerSetValue(&RxTimeoutTimer, timeout);
        //TimerStart(&RxTimeoutTimer);
    }

    if (rx_continuous) {
        sx1276_set_op_mode(dev, RFLR_OPMODE_RECEIVER);
    }
    else {
        sx1276_set_op_mode(dev, RFLR_OPMODE_RECEIVER_SINGLE);
    }
}

//XXX: removed
void sx1276_set_tx(sx1276_t *dev, uint32_t timeout)
{
    // TODO: use RIOT timers
    //TimerSetValue( &TxTimeoutTimer, timeout );

    switch (dev->settings.modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
        {
            if (dev->settings.lora.freq_hop_on) {
                sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK,
                                 RFLR_IRQFLAGS_RXTIMEOUT |
                                 RFLR_IRQFLAGS_RXDONE |
                                 RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                 RFLR_IRQFLAGS_VALIDHEADER |
                                    //RFLR_IRQFLAGS_TXDONE |
                                 RFLR_IRQFLAGS_CADDONE |
                                    //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                 RFLR_IRQFLAGS_CADDETECTED);

                // DIO0=TxDone, DIO2=FhssChangeChannel
                sx1276_reg_write(dev,
                                 REG_DIOMAPPING1,
                                 (sx1276_reg_read(dev, REG_DIOMAPPING1)
                                  & RFLR_DIOMAPPING1_DIO0_MASK
                                  & RFLR_DIOMAPPING1_DIO2_MASK)
                                 | RFLR_DIOMAPPING1_DIO0_01
                                 | RFLR_DIOMAPPING1_DIO2_00);
            }
            else {

            }
        }
        break;
    }

}

void sx1276_start_cad(sx1276_t *dev)
{
    switch (dev->settings.modem) {
        case MODEM_FSK:
        {

        }
        break;
        case MODEM_LORA:
        {
            sx1276_reg_write(dev, REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                             RFLR_IRQFLAGS_RXDONE |
                             RFLR_IRQFLAGS_PAYLOADCRCERROR |
                             RFLR_IRQFLAGS_VALIDHEADER |
                             RFLR_IRQFLAGS_TXDONE |
                                                                //RFLR_IRQFLAGS_CADDONE |
                             RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL   // |
                                                                //RFLR_IRQFLAGS_CADDETECTED
                             );

            // DIO3=CADDone
            sx1276_reg_write(dev,
                             REG_DIOMAPPING1,
                             (sx1276_reg_read(dev, REG_DIOMAPPING1)
                              & RFLR_DIOMAPPING1_DIO0_MASK) | RFLR_DIOMAPPING1_DIO0_00);

            sx1276_set_status(dev,  RF_CAD);
            sx1276_set_op_mode(dev, RFLR_OPMODE_CAD);
        }
        break;
        default:
            break;
    }
}

int16_t sx1276_read_rssi(sx1276_t *dev)
{
    int16_t rssi = 0;

    switch (dev->settings.modem) {
        case MODEM_FSK:
            rssi = -(sx1276_reg_read(dev, REG_RSSIVALUE) >> 1);
            break;
        case MODEM_LORA:
            if (dev->settings.channel > RF_MID_BAND_THRESH) {
                rssi = RSSI_OFFSET_HF + sx1276_reg_read(dev, REG_LR_RSSIVALUE);
            }
            else {
                rssi = RSSI_OFFSET_LF + sx1276_reg_read(dev, REG_LR_RSSIVALUE);
            }
            break;
        default:
            rssi = -1;
            break;
    }

    return rssi;
}

void sx1276_reset(sx1276_t *dev)
{
    /*
     * This reset scheme is complies with 7.2 chapter of the SX1276 datasheet
     *
     * 1. Set NReset pin to LOW for at least 100 us
     * 2. Set NReset in Hi-Z state
     * 3. Wait at least 5 milliseconds
     */

    gpio_init(dev->reset_pin, GPIO_OUT);

    /* Set reset pin to 0 */
    gpio_clear(dev->reset_pin);

    /* Wait 1 ms */
    xtimer_usleep(1000);

    /* Put reset pin in High-Z */
    gpio_init(dev->reset_pin, GPIO_OD);

    /* Wait 10 ms */
    xtimer_usleep(1000 * 10);
}

void sx1276_set_op_mode(sx1276_t *dev, uint8_t op_mode)
{
    static uint8_t op_mode_prev = 0;

    op_mode_prev = sx1276_reg_read(dev, REG_OPMODE) & ~RF_OPMODE_MASK;

    if (op_mode != op_mode_prev) {
        if (op_mode == RF_OPMODE_SLEEP) {
            sx1276_board_set_ant_sw_low_power(true);
        }
        else {
            sx1276_board_set_ant_sw_low_power(false);

            if (op_mode == RF_OPMODE_TRANSMITTER) {
                sx1276_board_set_ant_sw(1);
            }
            else {
                sx1276_board_set_ant_sw(0);
            }
        }

        printf("sx1276: RegOpMode (%02X) %02X -> %02X\n", op_mode, op_mode_prev, (op_mode_prev & RF_OPMODE_MASK) | op_mode);

        /* Replace previous mode value and setup new mode value */
        sx1276_reg_write(dev, REG_OPMODE, (op_mode_prev & RF_OPMODE_MASK) | op_mode);
        xtimer_usleep(1000 * 5); /* wait 5 milliseconds */

        printf("sx1276: After setup: %02X\n", sx1276_reg_read(dev, REG_OPMODE) & ~RF_OPMODE_MASK);
    }
}

void sx1276_set_max_payload_len(sx1276_t *dev, sx1276_radio_modems_t modem, uint8_t maxlen)
{
    sx1276_set_modem(dev, modem);

    switch (modem) {
        case MODEM_FSK:
            break;

        case MODEM_LORA:
            sx1276_reg_write(dev, REG_LR_PAYLOADMAXLENGTH, maxlen);
            break;
    }
}

/*
 * SPI Register routines
 */

void sx1276_reg_write(sx1276_t *dev, uint8_t addr, uint8_t data)
{
    sx1276_reg_write_burst(dev, addr, &data, 1);
}

uint8_t sx1276_reg_read(sx1276_t *dev, uint8_t addr)
{
    uint8_t data;

    sx1276_reg_read_burst(dev, addr, &data, 1);

    return data;
}

void sx1276_reg_write_burst(sx1276_t *dev, uint8_t addr, uint8_t *buffer,
                            uint8_t size)
{
    unsigned int cpsr;

    spi_acquire(dev->spi);
    cpsr = irq_disable();

    gpio_clear(dev->nss_pin);
    spi_transfer_regs(dev->spi, addr | 0x80, (char *) buffer, NULL, size);
    gpio_set(dev->nss_pin);

    irq_restore(cpsr);
    spi_release(dev->spi);
}

void sx1276_reg_read_burst(sx1276_t *dev, uint8_t addr, uint8_t *buffer,
                           uint8_t size)
{
    unsigned int cpsr;

    spi_acquire(dev->spi);
    cpsr = irq_disable();

    gpio_clear(dev->nss_pin);
    spi_transfer_regs(dev->spi, addr & 0x7F, NULL, (char *) buffer, size);
    gpio_set(dev->nss_pin);

    irq_restore(cpsr);
    spi_release(dev->spi);
}

void sx1276_write_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size)
{
    sx1276_reg_write_burst(dev, 0, buffer, size);
}

void sx1276_read_fifo(sx1276_t *dev, uint8_t *buffer, uint8_t size)
{
    sx1276_reg_read_burst(dev, 0, buffer, size);
}

/**
 * IRQ handlers
 */
void sx1276_on_dio0_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 0;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

void sx1276_on_dio1_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 1;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

void sx1276_on_dio2_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 2;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

void sx1276_on_dio3_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 3;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

void sx1276_on_dio4_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 4;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

void sx1276_on_dio5_isr(void *arg)
{
    msg_t msg;

    msg.content.value = 5;
    msg_try_send(&msg, ((sx1276_t *)arg)->dio_polling_thread_pid);
}

/**
 * State machine handlers
 */
void sx1276_on_timeout_isr(sx1276_t *dev)
{
    msg_t msg;
    sx1276_event_t event;

    switch (dev->settings.state) {
        case RF_RX_RUNNING:
            sx1276_set_status(dev,  RF_IDLE);
            //TimerStop( &RxTimeoutSyncWord ); // TODO: use RIOT timers

            event.type = RX_TIMEOUT;
            msg.content.ptr = (char *) &event;
            msg_try_send(&msg, dev->event_handler_thread_pid);
            break;

        case RF_TX_RUNNING:
            sx1276_set_status(dev,  RF_IDLE);
            event.type = TX_TIMEOUT;
            msg.content.ptr = (char *) &event;
            msg_try_send(&msg, dev->event_handler_thread_pid);
            break;
        default:
            break;
    }
}

void sx1276_on_dio0(void *arg)
{
    sx1276_t *dev = (sx1276_t *) arg;

    volatile uint8_t irq_flags = 0;

    switch (dev->settings.state) {
        case RF_RX_RUNNING:
            switch (dev->settings.modem) {
                case MODEM_LORA:
                {
                    int8_t snr = 0;

                    /* Clear IRQ */
                    sx1276_reg_write(dev,  REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);

                    irq_flags = sx1276_reg_read(dev,  REG_LR_IRQFLAGS);
                    if ((irq_flags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK) == RFLR_IRQFLAGS_PAYLOADCRCERROR) {
                        sx1276_reg_write(dev,  REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR); /* Clear IRQ */

                        if (!dev->settings.lora.rx_continuous) {
                            sx1276_set_status(dev,  RF_IDLE);
                        }

                        //TimerStop( &RxTimeoutTimer ); // TODO: RIOT timers

                        msg_t msg;
                        sx1276_event_t event;
                        event.type = RX_ERROR;
                        event.event_data = "CRC error";
                        msg.content.ptr = (char *) &event;
                        msg_try_send(&msg, dev->event_handler_thread_pid);

                        break;
                    }

                    sx1276_rx_packet_t packet;

                    packet.snr_value = sx1276_reg_read(dev,  REG_LR_PKTSNRVALUE);
                    if (packet.snr_value & 0x80) { /* The SNR is negative */
                        /* Invert and divide by 4 */
                        snr = ((~packet.snr_value + 1) & 0xFF) >> 2;
                        snr = -snr;
                    }
                    else {
                        /* Divide by 4 */
                        snr = (packet.snr_value & 0xFF) >> 2;
                    }

                    int16_t rssi = sx1276_reg_read(dev, REG_LR_PKTRSSIVALUE);
                    if (snr < 0) {
                        if (dev->settings.channel > RF_MID_BAND_THRESH) {
                            packet.rssi_value = RSSI_OFFSET_HF + rssi + (rssi >> 4) + snr;
                        }
                        else {
                            packet.rssi_value = RSSI_OFFSET_LF + rssi + (rssi >> 4) + snr;
                        }
                    }
                    else {
                        if (dev->settings.channel > RF_MID_BAND_THRESH) {
                            packet.rssi_value = RSSI_OFFSET_HF + rssi + (rssi >> 4);
                        }
                        else {
                            packet.rssi_value = RSSI_OFFSET_LF + rssi + (rssi >> 4);
                        }
                    }

                    packet.size = sx1276_reg_read(dev, REG_LR_RXNBBYTES);

                    if (!dev->settings.lora.rx_continuous) {
                        sx1276_set_status(dev,  RF_IDLE);
                    }

                    // TimerStop( &RxTimeoutTimer ); // TODO: RIOT timers

                    packet.content = (char *) malloc(packet.size);
                    if (packet.content == NULL) {
                        __BKPT(1); /* Out of memory */
                    }

                    sx1276_read_fifo(dev, (uint8_t *) packet.content, packet.size);

                    msg_t msg;
                    sx1276_event_t event;
                    event.type = RX_DONE;
                    event.event_data = &packet;
                    msg.content.ptr = (char *) &event;
                    msg_try_send(&msg, dev->event_handler_thread_pid);
                }
                break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            //TimerStop( &TxTimeoutTimer ); // TODO: RIOT timers
            sx1276_reg_write(dev, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE); /* Clear IRQ */
            sx1276_set_status(dev,  RF_IDLE);

            msg_t msg;
            sx1276_event_t event;
            event.type = TX_DONE;
            msg.content.ptr = (char *) &event;
            msg_try_send(&msg, dev->event_handler_thread_pid);
            break;
        default:
            break;
    }
}

void sx1276_on_dio1(void *arg)
{
    /* Get interrupt context */
    sx1276_t *dev = (sx1276_t *) arg;

    puts("[isr] DIO1 called"); // XXX: debug

    switch (dev->settings.state) {
        case RF_RX_RUNNING:
            switch (dev->settings.modem) {
                case MODEM_LORA:
                    // Sync time out
                    //TimerStop( &RxTimeoutTimer ); // TODO: RIOT timers

                    sx1276_set_status(dev,  RF_IDLE);

                    msg_t msg;
                    sx1276_event_t event;
                    event.type = RX_TIMEOUT;
                    msg.content.ptr = (char *) &event;
                    msg_try_send(&msg, dev->event_handler_thread_pid);
                    break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            break;
        default:
            break;
    }
}

void sx1276_on_dio2(void *arg)
{
    /* Get interrupt context */
    sx1276_t *dev = (sx1276_t *) arg;

    //puts("[isr] DIO2 called"); // XXX: debug

    switch (dev->settings.state) {
        case RF_RX_RUNNING:
            switch (dev->settings.modem) {
                case MODEM_LORA:
                    if (dev->settings.lora.freq_hop_on) {
                        /* Clear IRQ */
                        sx1276_reg_write(dev, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        msg_t msg;
                        sx1276_event_t event;
                        event.type = FHSS_CHANGE_CHANNEL;
                        uint32_t channel = sx1276_reg_read(dev, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK;
                        event.event_data = &channel;
                        msg.content.ptr = (char *) &event;
                        msg_try_send(&msg, dev->event_handler_thread_pid);
                    }

                    break;
                default:
                    break;
            }
            break;
        case RF_TX_RUNNING:
            switch (dev->settings.modem) {
                case MODEM_FSK:
                    break;
                case MODEM_LORA:
                    if (dev->settings.lora.freq_hop_on) {
                        /* Clear IRQ */
                        sx1276_reg_write(dev, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL);

                        msg_t msg;
                        sx1276_event_t event;
                        event.type = FHSS_CHANGE_CHANNEL;
                        uint32_t channel = sx1276_reg_read(dev, REG_LR_HOPCHANNEL) & RFLR_HOPCHANNEL_CHANNEL_MASK;
                        event.event_data = &channel;
                        msg.content.ptr = (char *) &event;
                        msg_try_send(&msg, dev->event_handler_thread_pid);
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}

void sx1276_on_dio3(void *arg)
{
    /* Get interrupt context */
    sx1276_t *dev = (sx1276_t *) arg;

    //puts("[isr] DIO3 called"); // XXX: debug

    switch (dev->settings.modem) {
        case MODEM_FSK:
            break;
        case MODEM_LORA:
            /* Clear IRQ */
            sx1276_reg_write(dev, REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE);

            /* Send event message */
            msg_t msg;
            sx1276_event_t event;
            event.type = CAD_DONE;
            bool result = (sx1276_reg_read(dev, REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_CADDETECTED) == RFLR_IRQFLAGS_CADDETECTED;
            event.event_data = &result;
            msg.content.ptr = (char *) &event;
            msg_try_send(&msg, dev->event_handler_thread_pid);
            break;
        default:
            break;
    }
}

void sx1276_on_dio4(void *arg)
{
    (void) arg;

    /* Empty (only LoRa related part is implemented) */
}

void sx1276_on_dio5(void *arg)
{
    (void) arg;

    /* Empty */
}

void *dio_polling_thread(void *arg)
{

    puts("sx1276: dio polling thread started");

    sx1276_t *dev = (sx1276_t *) arg;
    msg_init_queue(msg_queue, sizeof(msg_queue));

    msg_t msg;

    while (1) {
        msg_receive(&msg);

        printf("sx1276: received DIO #%d interrupt\n", (int) msg.content.value);

        uint32_t v = msg.content.value;
        switch (v) {
            case 0:
                sx1276_on_dio0(dev);
                break;

            case 1:
                sx1276_on_dio1(dev);
                break;

            case 2:
                sx1276_on_dio2(dev);
                break;

            case 3:
                sx1276_on_dio3(dev);
                break;

            case 4:
                sx1276_on_dio4(dev);
                break;

            case 5:
                sx1276_on_dio5(dev);
                break;
        }
    }

    return NULL;
}

int8_t sx1276_read_temp(sx1276_t *dev)
{
    int8_t temp = 0;
    uint8_t prev_op_mode;

    // Enable Temperature reading
    uint8_t imgcal = sx1276_reg_read(dev, REG_IMAGECAL);

    imgcal = (imgcal & RF_IMAGECAL_TEMPMONITOR_MASK) | RF_IMAGECAL_TEMPMONITOR_ON;
    sx1276_reg_write(dev, REG_IMAGECAL, imgcal);

    // save current Op Mode
    prev_op_mode = sx1276_reg_read(dev, REG_OPMODE);

    // put device in FSK RxSynth
    sx1276_reg_write(dev, REG_OPMODE, RF_OPMODE_SYNTHESIZER_RX);

    // Wait 1ms
    xtimer_usleep(1000);

    // Disable Temperature reading
    imgcal = sx1276_reg_read(dev, REG_IMAGECAL);
    imgcal = (imgcal & RF_IMAGECAL_TEMPMONITOR_MASK) | RF_IMAGECAL_TEMPMONITOR_OFF;
    sx1276_reg_write(dev, REG_IMAGECAL, imgcal);

    // Read temperature
    uint8_t reg_temp = sx1276_reg_read(dev, REG_TEMP);
    temp = reg_temp & 0x7F;

    if ((reg_temp & 0x80) == 0x80) {
        temp *= -1;
    }

    // Reload previous Op Mode
    sx1276_reg_write(dev, REG_OPMODE, prev_op_mode);

    return temp;
}
