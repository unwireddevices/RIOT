/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sx128x Semtech SX1280 and SX1281 radios driver
 * @ingroup     drivers_netdev
 * @brief       Driver for Semtech SX1280 and SX1281 radios.
 *
 * This module contains the driver for radio devices of the Semtech SX128x
 * series (SX1280 and SX1281).
 * Only LoRa long range modem is supported at the moment.
 *
 * SX128x modules are designed to be used in the ISM radio frequency (RF) band.
 * This RF band depends on different regional regulations worldwide.
 * Be careful to configure the device to use a RF frequency allowed in your
 * region.
 *
 *
 * @{
 *
 * @file
 * @brief       Public interface for SX128X driver
 * 
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef _SX128X_H_
#define _SX128X_H_

#include "lptimer.h"
#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enables/disables driver debug features
 */
#define SX1280_DEBUG                                    (0)

/**
 * @brief Hardware IO IRQ callback function definition
 */
typedef void (*dio_irq_handler)(void *arg);

/**
 * @brief Provides the frequency of the chip running on the radio and the frequency step
 *
 * @remark These defines are used for computing the frequency divider to set the RF frequency
 */
#define SX128X_XTAL_FREQ                                (52000000UL)
#define SX128X_FREQ_STEP                                ((double)(SX128X_XTAL_FREQ/pow(2.0, 18.0)))

/**
 * @brief Compensation delay for SetAutoTx/Rx functions in microseconds
 */
#define AUTO_RX_TX_OFFSET                               (33)





/**
 * @brief   LoRa configuration structure.
 */
// typedef struct {
//     uint16_t preamble_len;              /**< Length of preamble header */
//     int8_t   power;                     /**< Signal power */
//     uint8_t  bandwidth;                 /**< Signal bandwidth */
//     uint8_t  datarate;                  /**< Spreading factor rate, e.g datarate */
//     uint8_t  coderate;                  /**< Error coding rate */
//     uint8_t  freq_hop_period;           /**< Frequency hop period */
//     uint8_t  flags;                     /**< Boolean flags */
//     uint32_t rx_timeout;                /**< RX timeout in milliseconds */
//     uint32_t tx_timeout;                /**< TX timeout in milliseconds */
// } sx128x_lora_settings_t;

/**
 * @brief   Radio settings.
 */
// typedef struct {
//     uint32_t channel;                   /**< Radio channel */
//     uint8_t state;                      /**< Radio state */
//     uint8_t modem;                      /**< Driver model (FSK or LoRa) */
//     sx128x_lora_settings_t lora;        /**< LoRa settings */
// } sx128x_radio_settings_t;

typedef enum {
    SX127X_MODEM_SX1280 = 0,
    SX127X_MODEM_SX1281 = 1,
} sx128x_modem_chip_t;

/**
 * @brief   SX128X internal data.
 */
// typedef struct {
//     /* Data that will be passed to events handler in application */
//     lptimer_t tx_timeout_timer;         /**< TX operation timeout timer */
//     lptimer_t rx_timeout_timer;         /**< RX operation timeout timer */
//     uint32_t last_channel;              /**< Last channel in frequency hopping sequence */
//     sx128x_modem_chip_t modem_chip;     /**< Modem model */
//     bool is_last_cad_success;           /**< Sign of success of last CAD operation (activity detected) */
// } sx128x_internal_t;

/**
 * @brief   SX128X hardware and global parameters.
 */
typedef struct {
    spi_t      spi_dev;                 /**< SPI device          */
    spi_clk_t  spi_speed;               /**< SPI clock speeds    */
    spi_mode_t spi_mode;                /**< SPI modes           */
    gpio_t     nss_pin;                 /**< SPI NSS pin         */
    gpio_t     reset_pin;               /**< Reset pin           */
    gpio_t     busy_pin;                /**< Busy pin            */
    gpio_t     dio1_pin;                /**< Interrupt line DIO1 */
    gpio_t     dio2_pin;                /**< Interrupt line DIO2 */
    gpio_t     dio3_pin;                /**< Interrupt line DIO3 */
} sx128x_params_t;

/**
 * @brief   SX128X IRQ flags.
 */
typedef uint8_t sx128x_flags_t;

/**
 * @brief   SX128X device descriptor.
 * @extends netdev_t
 */
typedef struct {
    // netdev_t netdev;                    /**< Netdev parent struct */
    // sx128x_radio_settings_t settings;   /**< Radio settings */
    sx128x_params_t params;             /**< Device driver parameters */
    dio_irq_handler cb;                     /**< callback */
    void *arg;                          /**< callback param */
    // sx128x_internal_t _internal;        /**< Internal sx128x data used within the driver */
    // sx128x_flags_t irq;                 /**< Device IRQ flags */
} sx128x_t;

/**
 * @brief Selector values to configure LNA regime
 */
typedef enum {
    SX128X_LNA_LOW_POWER_MODE,              /**< Allow maximum efficiency of sx1280 (default) */
    SX128X_LNA_HIGH_SENSITIVITY_MODE,       /**< Allow to use highest three steps of LNA gain and increase current consumption */
} sx128x_radio_lna_settings_t;

/**
 * @brief Structure describing the radio status
 */
typedef union
{
    /**
     * @brief Structure of the radio status
     */
    struct
    {
        uint8_t cpu_busy   : 1;             /**< Flag for CPU radio busy */
        uint8_t dma_busy   : 1;             /**< Flag for DMA busy */
        uint8_t cmd_status : 3;             /**< Command status */
        uint8_t chip_mode  : 3;             /**< Chip mode */
    } fields;

    /**
     * @brief Serialized radio status
     */
    uint8_t Value;
} sx128x_radio_status_t;

/**
 * @brief Represents the states of the radio
 */
typedef enum {
    SX128X_RF_IDLE         = 0x00,          /**< The radio is idle */
    SX128X_RF_RX_RUNNING,                   /**< The radio is in reception state */
    SX128X_RF_TX_RUNNING,                   /**< The radio is in transmission state */
    SX128X_RF_CAD,                          /**< The radio is doing channel activity detection */
} sx128x_radio_states_t;

/**
 * @brief Represents the operating mode the radio is actually running
 */
typedef enum {
    SX128X_MODE_SLEEP        = 0x00,        /**< The radio is in sleep mode */
    SX128X_MODE_STDBY_RC,                   /**< The radio is in standby mode with RC oscillator */
    SX128X_MODE_STDBY_XOSC,                 /**< The radio is in standby mode with XOSC oscillator */
    SX128X_MODE_FS,                         /**< The radio is in frequency synthesis mode */
    SX128X_MODE_TX,                         /**< The radio is in transmit mode */
    SX128X_MODE_RX,                         /**< The radio is in receive mode */
    SX128X_MODE_CAD                         /**< The radio is in channel activity detection mode */
} sx128x_radio_operating_modes_t;

/**
 * @brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum {
    SX128X_STDBY_RC   = 0x00,
    SX128X_STDBY_XOSC = 0x01,
} sx128x_radio_standby_modes_t;

/**
 * @brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum {
    SX128X_USE_LDO  = 0x00,                 /**< Use LDO (default value) */
    SX128X_USE_DCDC = 0x01,                 /**< Use DCDC */
} sx128x_radio_regulator_modes_t;

/**
 * @brief Represents the possible packet type (i.e. modem) used
 */
typedef enum {
    SX128X_PACKET_TYPE_GFSK     = 0x00,
    SX128X_PACKET_TYPE_LORA,
    SX128X_PACKET_TYPE_RANGING,
    SX128X_PACKET_TYPE_FLRC,
    SX128X_PACKET_TYPE_BLE,
    SX128X_PACKET_TYPE_NONE     = 0x0F,
} sx128x_radio_packet_types_t;

/**
 * @brief Represents the ramping time for power amplifier
 */
typedef enum {
    SX128X_RADIO_RAMP_02_US = 0x00,
    SX128X_RADIO_RAMP_04_US = 0x20,
    SX128X_RADIO_RAMP_06_US = 0x40,
    SX128X_RADIO_RAMP_08_US = 0x60,
    SX128X_RADIO_RAMP_10_US = 0x80,
    SX128X_RADIO_RAMP_12_US = 0xA0,
    SX128X_RADIO_RAMP_16_US = 0xC0,
    SX128X_RADIO_RAMP_20_US = 0xE0,
} sx128x_radio_ramp_times_t;

/**
 * @brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum {
    SX128X_LORA_CAD_01_SYMBOL = 0x00,
    SX128X_LORA_CAD_02_SYMBOL = 0x20,
    SX128X_LORA_CAD_04_SYMBOL = 0x40,
    SX128X_LORA_CAD_08_SYMBOL = 0x60,
    SX128X_LORA_CAD_16_SYMBOL = 0x80,
} sx128x_radio_lora_cad_symbols_t;

/**
 * @brief Represents the possible combinations of bitrate and bandwidth for
 *        GFSK and BLE packet types
 *
 * The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef enum {
    SX128X_GFSK_BLE_BR_2_000_BW_2_4 = 0x04,
    SX128X_GFSK_BLE_BR_1_600_BW_2_4 = 0x28,
    SX128X_GFSK_BLE_BR_1_000_BW_2_4 = 0x4C,
    SX128X_GFSK_BLE_BR_1_000_BW_1_2 = 0x45,
    SX128X_GFSK_BLE_BR_0_800_BW_2_4 = 0x70,
    SX128X_GFSK_BLE_BR_0_800_BW_1_2 = 0x69,
    SX128X_GFSK_BLE_BR_0_500_BW_1_2 = 0x8D,
    SX128X_GFSK_BLE_BR_0_500_BW_0_6 = 0x86,
    SX128X_GFSK_BLE_BR_0_400_BW_1_2 = 0xB1,
    SX128X_GFSK_BLE_BR_0_400_BW_0_6 = 0xAA,
    SX128X_GFSK_BLE_BR_0_250_BW_0_6 = 0xCE,
    SX128X_GFSK_BLE_BR_0_250_BW_0_3 = 0xC7,
    SX128X_GFSK_BLE_BR_0_125_BW_0_3 = 0xEF,
} sx128x_radio_gfsk_ble_bitrates_t;

/**
 * @brief Represents the modulation index used in GFSK and BLE packet
 *        types
 */
typedef enum {
    SX128X_GFSK_BLE_MOD_IND_0_35 =  0,
    SX128X_GFSK_BLE_MOD_IND_0_50 =  1,
    SX128X_GFSK_BLE_MOD_IND_0_75 =  2,
    SX128X_GFSK_BLE_MOD_IND_1_00 =  3,
    SX128X_GFSK_BLE_MOD_IND_1_25 =  4,
    SX128X_GFSK_BLE_MOD_IND_1_50 =  5,
    SX128X_GFSK_BLE_MOD_IND_1_75 =  6,
    SX128X_GFSK_BLE_MOD_IND_2_00 =  7,
    SX128X_GFSK_BLE_MOD_IND_2_25 =  8,
    SX128X_GFSK_BLE_MOD_IND_2_50 =  9,
    SX128X_GFSK_BLE_MOD_IND_2_75 = 10,
    SX128X_GFSK_BLE_MOD_IND_3_00 = 11,
    SX128X_GFSK_BLE_MOD_IND_3_25 = 12,
    SX128X_GFSK_BLE_MOD_IND_3_50 = 13,
    SX128X_GFSK_BLE_MOD_IND_3_75 = 14,
    SX128X_GFSK_BLE_MOD_IND_4_00 = 15,
} sx128x_radio_gfsk_ble_mod_indexes_t;

/**
 * @brief Represents the possible combination of bitrate and bandwidth for FLRC
 *        packet type
 *
 * The bitrate is in Mb/s and the bitrate in MHz
 */
typedef enum {
    SX128X_FLRC_BR_2_600_BW_2_4 = 0x04,
    SX128X_FLRC_BR_2_080_BW_2_4 = 0x28,
    SX128X_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX128X_FLRC_BR_1_040_BW_1_2 = 0x69,
    SX128X_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX128X_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX128X_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX128X_FLRC_BR_0_260_BW_0_3 = 0xEB,
} sx128x_radio_flrc_bitrates_t;

/**
 * @brief Represents the possible values for coding rate parameter in FLRC
 *        packet type
 */
typedef enum {
    SX128X_FLRC_CR_1_2 = 0x00,
    SX128X_FLRC_CR_3_4 = 0x02,
    SX128X_FLRC_CR_1_0 = 0x04,
} sx128x_radio_flrc_coding_rates_t;

/**
 * @brief Represents the modulation shaping parameter for GFSK, FLRC and BLE
 *        packet types
 */
typedef enum {
    SX128X_RADIO_MOD_SHAPING_BT_OFF = 0x00,         /**< No filtering */
    SX128X_RADIO_MOD_SHAPING_BT_1_0 = 0x10,
    SX128X_RADIO_MOD_SHAPING_BT_0_5 = 0x20,
} sx128x_radio_mod_shapings_t;

/**
 * @brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum {
    SX128X_LORA_SF5  = 0x50,
    SX128X_LORA_SF6  = 0x60,
    SX128X_LORA_SF7  = 0x70,
    SX128X_LORA_SF8  = 0x80,
    SX128X_LORA_SF9  = 0x90,
    SX128X_LORA_SF10 = 0xA0,
    SX128X_LORA_SF11 = 0xB0,
    SX128X_LORA_SF12 = 0xC0,
} sx128x_radio_lora_spreading_factors_t;

/**
 * @brief Represents the bandwidth values for LORA packet type
 */
typedef enum {
    SX128X_LORA_BW_0200 = 0x34,
    SX128X_LORA_BW_0400 = 0x26,
    SX128X_LORA_BW_0800 = 0x18,
    SX128X_LORA_BW_1600 = 0x0A,
} sx128x_radio_lora_bandwidths_t;

/**
 * @brief Represents the coding rate values for LORA packet type
 */
typedef enum {
    SX128X_LORA_CR_4_5    = 0x01,
    SX128X_LORA_CR_4_6    = 0x02,
    SX128X_LORA_CR_4_7    = 0x03,
    SX128X_LORA_CR_4_8    = 0x04,
    SX128X_LORA_CR_LI_4_5 = 0x05,
    SX128X_LORA_CR_LI_4_6 = 0x06,
    SX128X_LORA_CR_LI_4_7 = 0x07,
} sx128x_radio_lora_coding_rates_t;

/**
 * @brief Represents the preamble length values for GFSK and FLRC packet
 *        types
 */
typedef enum {
    SX128X_PREAMBLE_LENGTH_04_BITS = 0x00,              /**< Preamble length: 04 bits */
    SX128X_PREAMBLE_LENGTH_08_BITS = 0x10,              /**< Preamble length: 08 bits */
    SX128X_PREAMBLE_LENGTH_12_BITS = 0x20,              /**< Preamble length: 12 bits */
    SX128X_PREAMBLE_LENGTH_16_BITS = 0x30,              /**< Preamble length: 16 bits */
    SX128X_PREAMBLE_LENGTH_20_BITS = 0x40,              /**< Preamble length: 20 bits */
    SX128X_PREAMBLE_LENGTH_24_BITS = 0x50,              /**< Preamble length: 24 bits */
    SX128X_PREAMBLE_LENGTH_28_BITS = 0x60,              /**< Preamble length: 28 bits */
    SX128X_PREAMBLE_LENGTH_32_BITS = 0x70,              /**< Preamble length: 32 bits */
} sx128x_radio_preamble_lengths_t;

/**
 * @brief Represents the SyncWord length for FLRC packet type
 */
typedef enum {
    SX128X_FLRC_NO_SYNCWORD            = 0x00,
    SX128X_FLRC_SYNCWORD_LENGTH_4_BYTE = 0x04,
} sx128x_radio_flrc_sync_word_lengths_t;

/**
 * @brief The length of sync words for GFSK packet type
 */
typedef enum {
    SX128X_GFSK_SYNCWORD_LENGTH_1_BYTE = 0x00,          /**< Sync word length: 1 byte  */
    SX128X_GFSK_SYNCWORD_LENGTH_2_BYTE = 0x02,          /**< Sync word length: 2 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_3_BYTE = 0x04,          /**< Sync word length: 3 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_4_BYTE = 0x06,          /**< Sync word length: 4 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_5_BYTE = 0x08,          /**< Sync word length: 5 bytes */
} sx128x_radio_sync_word_lengths_t;

/**
 * @brief Represents the possible combinations of SyncWord correlators
 *        activated for GFSK and FLRC packet types
 */
typedef enum {
    SX128X_RADIO_RX_MATCH_SYNCWORD_OFF   = 0x00,        /**< No correlator turned on, i.e. do not search for SyncWord */
    SX128X_RADIO_RX_MATCH_SYNCWORD_1     = 0x10,
    SX128X_RADIO_RX_MATCH_SYNCWORD_2     = 0x20,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_2   = 0x30,
    SX128X_RADIO_RX_MATCH_SYNCWORD_3     = 0x40,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_3   = 0x50,
    SX128X_RADIO_RX_MATCH_SYNCWORD_2_3   = 0x60,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_2_3 = 0x70,
} sx128x_radio_sync_word_rx_matchs_t;

/**
 *  @brief Radio packet length mode for GFSK and FLRC packet types
 */
typedef enum {
    SX128X_RADIO_PACKET_FIXED_LENGTH    = 0x00,         /**< The packet is known on both sides, no header included in the packet */
    SX128X_RADIO_PACKET_VARIABLE_LENGTH = 0x20,         /**< The packet is on variable size, header included */
} sx128x_radio_packet_length_modes_t;

/**
 * @brief Represents the CRC length for GFSK and FLRC packet types
 *
 * @warning Not all configurations are available for both GFSK and FLRC
 *          packet type. Refer to the datasheet for possible configuration.
 */
typedef enum {
    SX128X_RADIO_CRC_OFF     = 0x00,                    /**< No CRC in use */
    SX128X_RADIO_CRC_1_BYTES = 0x10,
    SX128X_RADIO_CRC_2_BYTES = 0x20,
    SX128X_RADIO_CRC_3_BYTES = 0x30,
} sx128x_radio_crc_types_t;

/**
 * @brief Radio whitening mode activated or deactivated for GFSK, FLRC and
 *        BLE packet types
 */
typedef enum {
    SX128X_RADIO_WHITENING_ON  = 0x00,
    SX128X_RADIO_WHITENING_OFF = 0x08,
} sx128x_radio_whitening_modes_t;

/**
 * @brief Holds the packet length mode of a LORA packet type
 */
typedef enum {
    SX128X_LORA_PACKET_VARIABLE_LENGTH = 0x00,          /**< The packet is on variable size, header included */
    SX128X_LORA_PACKET_FIXED_LENGTH    = 0x80,          /**< The packet is known on both sides, no header included in the packet */
    SX128X_LORA_PACKET_EXPLICIT        = SX128X_LORA_PACKET_VARIABLE_LENGTH,
    SX128X_LORA_PACKET_IMPLICIT        = SX128X_LORA_PACKET_FIXED_LENGTH,
} sx128x_radio_lora_packet_lengths_modes_t;

/**
 * @brief Represents the CRC mode for LORA packet type
 */
typedef enum {
    SX128X_LORA_CRC_ON  = 0x20,                         /**< CRC activated */
    SX128X_LORA_CRC_OFF = 0x00,                         /**< CRC not used */
} sx128x_radio_lora_crc_modes_t;

/**
 * @brief Represents the IQ mode for LORA packet type
 */
typedef enum {
    SX128X_LORA_IQ_NORMAL   = 0x40,
    SX128X_LORA_IQ_INVERTED = 0x00,
} sx128x_radio_lora_iq_modes_t;

/**
 * @brief Represents the length of the ID to check in ranging operation
 */
typedef enum {
    SX128X_RANGING_IDCHECK_LENGTH_08_BITS = 0x00,
    SX128X_RANGING_IDCHECK_LENGTH_16_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_24_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_32_BITS,
} sx128x_radio_ranging_id_check_lengths_t;

/**
 * @brief Represents the result type to be used in ranging operation
 */
typedef enum {
    SX128X_RANGING_RESULT_RAW      = 0x00,
    SX128X_RANGING_RESULT_AVERAGED = 0x01,
    SX128X_RANGING_RESULT_DEBIASED = 0x02,
    SX128X_RANGING_RESULT_FILTERED = 0x03,
} sx128x_radio_ranging_result_types_t;

/**
 * @brief Represents the connection state for BLE packet type
 */
typedef enum {
    SX128X_BLE_PAYLOAD_LENGTH_MAX_31_BYTES  = 0x00,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_37_BYTES  = 0x20,
    SX128X_BLE_TX_TEST_MODE                 = 0x40,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_255_BYTES = 0x80,
} sx128x_radio_ble_connection_states_t;

/**
 * @brief Represents the CRC field length for BLE packet type
 */
typedef enum {
    SX128X_BLE_CRC_OFF = 0x00,
    SX128X_BLE_CRC_3B  = 0x10,
} sx128x_radio_ble_crc_fields_t;

/**
 * @brief Represents the specific packets to use in BLE packet type
 */
typedef enum {
    SX128X_BLE_PRBS_9       = 0x00,                     /**< Pseudo Random Binary Sequence based on 9th degree polynomial */
    SX128X_BLE_PRBS_15      = 0x0C,                     /**< Pseudo Random Binary Sequence based on 15th degree polynomial */
    SX128X_BLE_EYELONG_1_0  = 0x04,                     /**< Repeated '11110000' sequence */
    SX128X_BLE_EYELONG_0_1  = 0x18,                     /**< Repeated '00001111' sequence */
    SX128X_BLE_EYESHORT_1_0 = 0x08,                     /**< Repeated '10101010' sequence */
    SX128X_BLE_EYESHORT_0_1 = 0x1C,                     /**< Repeated '01010101' sequence */
    SX128X_BLE_ALL_1        = 0x10,                     /**< Repeated '11111111' sequence */
    SX128X_BLE_ALL_0        = 0x14,                     /**< Repeated '00000000' sequence */
} sx128x_radio_ble_packet_types_t;

/**
 * @brief Represents the interruption masks available for the radio
 *
 * @remark Note that not all these interruptions are available for all packet types
 */
typedef enum {
    SX128X_IRQ_RADIO_NONE                       = 0x0000,
    SX128X_IRQ_TX_DONE                          = 0x0001,
    SX128X_IRQ_RX_DONE                          = 0x0002,
    SX128X_IRQ_SYNCWORD_VALID                   = 0x0004,
    SX128X_IRQ_SYNCWORD_ERROR                   = 0x0008,
    SX128X_IRQ_HEADER_VALID                     = 0x0010,
    SX128X_IRQ_HEADER_ERROR                     = 0x0020,
    SX128X_IRQ_CRC_ERROR                        = 0x0040,
    SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE      = 0x0080,
    SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED  = 0x0100,
    SX128X_IRQ_RANGING_MASTER_RESULT_VALID      = 0x0200,
    SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT    = 0x0400,
    SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID      = 0x0800,
    SX128X_IRQ_CAD_DONE                         = 0x1000,
    SX128X_IRQ_CAD_ACTIVITY_DETECTED            = 0x2000,
    SX128X_IRQ_RX_TX_TIMEOUT                    = 0x4000,
    SX128X_IRQ_PREAMBLE_DETECTED                = 0x8000,
    SX128X_IRQ_RADIO_ALL                        = 0xFFFF,
} sx128x_radio_irq_masks_t;

/**
 * @brief Represents the digital input/output of the radio
 */
typedef enum {
    SX128X_RADIO_DIO1 = 0x02,
    SX128X_RADIO_DIO2 = 0x04,
    SX128X_RADIO_DIO3 = 0x08,
} sx128x_radio_dios_t;

/**
 * @brief Represents the role of the radio during ranging operations
 */
typedef enum {
    SX128X_RADIO_RANGING_ROLE_SLAVE  = 0x00,
    SX128X_RADIO_RANGING_ROLE_MASTER = 0x01,
} sx128x_radio_ranging_roles_t;

/**
 * @brief Represents the tick size available for Rx/Tx timeout operations
 */
typedef enum {
    SX128X_RADIO_TICK_SIZE_0015_US = 0x00,
    SX128X_RADIO_TICK_SIZE_0062_US = 0x01,
    SX128X_RADIO_TICK_SIZE_1000_US = 0x02,
    SX128X_RADIO_TICK_SIZE_4000_US = 0x03,
} sx128x_radio_tick_sizes_t;

/**
 * @brief Represents an amount of time measurable by the radio clock
 *
 * @code
 * Time = Step * NbSteps
 * Example:
 * Step = SX128X_RADIO_TICK_SIZE_4000_US( 4 ms )
 * NbSteps = 1000
 * Time = 4e-3 * 1000 = 4 seconds
 * @endcode
 */
typedef struct {
    sx128x_radio_tick_sizes_t step;                                  /**< The step of ticktime */
    /**
     * @brief The number of steps for ticktime
     * Special values are:
     *     - 0x0000 for single mode
     *     - 0xFFFF for continuous mode
     */
    uint16_t nb_steps;
} sx128x_tick_time_t;

/**
 * @brief RX_TX_CONTINUOUS and RX_TX_SINGLE are two particular values for TickTime.
 * The former keep the radio in Rx or Tx mode, even after successfull reception
 * or transmission. It should never generate Timeout interrupt.
 * The later let the radio enought time to make one reception or transmission.
 * No Timeout interrupt is generated, and the radio fall in StandBy mode after
 * reception or transmission.
 */
#define SX128X_RX_TX_CONTINUOUS                         (sx1280_tick_time_t){RADIO_TICK_SIZE_0015_US, 0xFFFF}
#define SX128X_RX_TX_SINGLE                             (sx1280_tick_time_t){RADIO_TICK_SIZE_0015_US, 0}

/*!
 * \brief The type describing the modulation parameters for every packet types
 */
typedef struct
{
    sx128x_radio_packet_types_t packet_type;                            //!< Packet to which the modulation parameters are referring to.
//    union
    struct {
        /**
         * @brief Holds the GFSK modulation parameters
         *
         * In GFSK modulation, the bit-rate and bandwidth are linked together. 
         * In this structure, its values are set using the same token.
         */
        struct {
            sx128x_radio_gfsk_ble_bitrates_t    bitrate_bandwidth;      //!< The bandwidth and bit-rate values for BLE and GFSK modulations
            sx128x_radio_gfsk_ble_mod_indexes_t modulation_index;       //!< The coding rate for BLE and GFSK modulations
            sx128x_radio_mod_shapings_t         modulation_shaping;     //!< The modulation shaping for BLE and GFSK modulations
        } gfsk;
        /**
         * @brief Holds the LORA modulation parameters
         *
         * LORA modulation is defined by Spreading Factor (SF), Bandwidth and Coding Rate
         */
        struct {
            sx128x_radio_lora_spreading_factors_t spreading_factor;     //!< Spreading Factor for the LORA modulation
            sx128x_radio_lora_bandwidths_t        bandwidth;            //!< Bandwidth for the LORA modulation
            sx128x_radio_lora_coding_rates_t      coding_rate;          //!< Coding rate for the LORA modulation
        } lora;
        /**
         * @brief Holds the FLRC modulation parameters
         *
         * In FLRC modulation, the bit-rate and bandwidth are linked together. 
         * In this structure, its values are set using the same token.
         */
        struct {
            sx128x_radio_flrc_bitrates_t     bitrate_bandwidth;         //!< The bandwidth and bit-rate values for FLRC modulation
            sx128x_radio_flrc_coding_rates_t coding_rate;               //!< The coding rate for FLRC modulation
            sx128x_radio_mod_shapings_t      modulation_shaping;        //!< The modulation shaping for FLRC modulation
        } flrc;
        /**
         * @brief Holds the BLE modulation parameters
         *
         * In BLE modulation, the bit-rate and bandwidth are linked together. 
         * In this structure, its values are set using the same token.
         */
        struct {
            sx128x_radio_gfsk_ble_bitrates_t    bitrate_bandwidth;      //!< The bandwidth and bit-rate values for BLE and GFSK modulations
            sx128x_radio_gfsk_ble_mod_indexes_t modulation_index;       //!< The coding rate for BLE and GFSK modulations
            sx128x_radio_mod_shapings_t         modulation_shaping;     //!< The modulation shaping for BLE and GFSK modulations
        } ble;
    } params;                                                           //!< Holds the modulation parameters structure
} sx128x_modulation_params_t;

/**
 * @brief Structure describing the error codes for callback functions
 */
typedef enum {
    SX128X_IRQ_HEADER_ERROR_CODE           = 0x00,
    SX128X_IRQ_SYNCWORD_ERROR_CODE,
    SX128X_IRQ_CRC_ERROR_CODE,
    SX128X_IRQ_RANGING_ON_LORA_ERROR_CODE,
} sx128x_irq_error_code_t;

/**
 * @brief Structure describing the ranging codes for callback functions
 */
typedef enum {
    SX128X_IRQ_RANGING_SLAVE_ERROR_CODE    = 0x00,
    SX128X_IRQ_RANGING_SLAVE_VALID_CODE,
    SX128X_IRQ_RANGING_MASTER_ERROR_CODE,
    SX128X_IRQ_RANGING_MASTER_VALID_CODE,
} sx128x_irq_ranging_code_t;

/**
 * @brief The radio callbacks structure
 * Holds function pointers to be called on radio interrupts
 */
typedef struct {
    void (*tx_done)(void);                                  //!< Pointer to a function run on successful transmission
    void (*rx_done)(void);                                  //!< Pointer to a function run on successful reception
    void (*rx_sync_word_done)(void);                        //!< Pointer to a function run on successful SyncWord reception
    void (*rx_header_done)(void);                           //!< Pointer to a function run on successful Header reception
    void (*tx_timeout)(void);                               //!< Pointer to a function run on transmission timeout
    void (*rx_timeout)(void);                               //!< Pointer to a function run on reception timeout
    void (*rx_error)(sx128x_irq_error_code_t err_code);     //!< Pointer to a function run on reception error
    void (*ranging_done)(sx128x_irq_ranging_code_t val);    //!< Pointer to a function run on ranging terminated
    void (*cad_done)(bool cad_flag);                        //!< Pointer to a function run on channel activity detected
} sx128x_radio_callbacks_t;

/**
 * @brief The type describing the packet parameters for every packet types
 */
typedef struct
{
    sx128x_radio_packet_types_t packet_type;                //!< Packet to which the packet parameters are referring to.
//    union
    struct {
        /**
         * @brief Holds the GFSK packet parameters
         */
        struct {
            sx128x_radio_preamble_lengths_t    preamble_length;         //!< The preamble length for GFSK packet type
            sx128x_radio_sync_word_lengths_t   sync_word_length;        //!< The synchronization word length for GFSK packet type
            sx128x_radio_sync_word_rx_matchs_t sync_word_match;         //!< The synchronization correlator to use to check synchronization word
            sx128x_radio_packet_length_modes_t header_type;             //!< If the header is explicit, it will be transmitted in the GFSK packet. If the header is implicit, it will not be transmitted
            uint8_t                            payload_length;          //!< Size of the payload in the GFSK packet
            sx128x_radio_crc_types_t           crc_length;              //!< Size of the CRC block in the GFSK packet
            sx128x_radio_whitening_modes_t     whitening;               //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
        } gfsk;
        /**
         * @brief Holds the LORA packet parameters
         */
        struct {
            uint8_t                                  preamble_length;    //!< The preamble length is the number of LORA symbols in the preamble. To set it, use the following formula @code Number of symbols = PreambleLength[3:0] * ( 2^PreambleLength[7:4] ) @endcode
            sx128x_radio_lora_packet_lengths_modes_t header_type;        //!< If the header is explicit, it will be transmitted in the LORA packet. If the header is implicit, it will not be transmitted
            uint8_t                                  payload_length;     //!< Size of the payload in the LORA packet
            sx128x_radio_lora_crc_modes_t            crc_mode;           //!< Size of CRC block in LORA packet
            sx128x_radio_lora_iq_modes_t             invert_iq;          //!< Allows to swap IQ for LORA packet
        } lora;
        /**
         * @brief Holds the FLRC packet parameters
         */
        struct {
            sx128x_radio_preamble_lengths_t       preamble_length;      //!< The preamble length for FLRC packet type
            sx128x_radio_flrc_sync_word_lengths_t sync_word_length;     //!< The synchronization word length for FLRC packet type
            sx128x_radio_sync_word_rx_matchs_t    sync_word_match;      //!< The synchronization correlator to use to check synchronization word
            sx128x_radio_packet_length_modes_t    header_type;          //!< If the header is explicit, it will be transmitted in the FLRC packet. If the header is implicit, it will not be transmitted.
            uint8_t                               payload_length;       //!< Size of the payload in the FLRC packet
            sx128x_radio_crc_types_t              crc_length;           //!< Size of the CRC block in the FLRC packet
            sx128x_radio_whitening_modes_t        whitening;            //!< Usage of whitening on payload and CRC blocks plus header block if header type is variable
        } flrc;
        /**
         * @brief Holds the BLE packet parameters
         */
        struct {
            sx128x_radio_ble_connection_states_t connection_state;      //!< The BLE state
            sx128x_radio_ble_crc_fields_t        crc_field;             //!< Size of the CRC block in the BLE packet
            sx128x_radio_ble_packet_types_t      ble_packet_type;       //!< Special BLE packet types
            sx128x_radio_whitening_modes_t       whitening;             //!< Usage of whitening on PDU and CRC blocks of BLE packet
        } ble;
    } params;                                                           //!< Holds the packet parameters structure
} sx128x_packet_params_t;

/**
 * @brief Represents the packet status for every packet type
 */
typedef struct {
    sx128x_radio_packet_types_t packet_type;           //!< Packet to which the packet status are referring to.
    union {
        struct {
            int8_t rssi_avg;                            //!< The averaged RSSI
            int8_t rssi_sync;                           //!< The RSSI measured on last packet
            struct {
                bool sync_error            :1;          //!< SyncWord error on last packet
                bool length_error          :1;          //!< Length error on last packet
                bool crc_error             :1;          //!< CRC error on last packet
                bool abort_error           :1;          //!< Abort error on last packet
                bool header_received       :1;          //!< Header received on last packet
                bool packet_received       :1;          //!< Packet received
                bool packet_controler_busy :1;          //!< Packet controller busy
            } error_status;                             //!< The error status Byte
            struct {
                bool rx_no_ack       :1;                //!< No acknowledgment received for Rx with variable length packets
                bool packet_sent     :1;                //!< Packet sent, only relevant in Tx mode
            }tx_rx_status;                              //!< The Tx/Rx status Byte
            uint8_t sync_addr_status :3;                //!< The id of the correlator who found the packet
        } gfsk;
        struct {
            int8_t rssi_pkt;                            //!< The RSSI of the last packet
            int8_t snr_pkt;                             //!< The SNR of the last packet
            struct {
                bool sync_error            :1;          //!< SyncWord error on last packet
                bool length_error          :1;          //!< Length error on last packet
                bool crc_error             :1;          //!< CRC error on last packet
                bool abort_error           :1;          //!< Abort error on last packet
                bool header_received       :1;          //!< Header received on last packet
                bool packet_received       :1;          //!< Packet received
                bool packet_controler_busy :1;          //!< Packet controller busy
            } error_status;                             //!< The error status Byte
            struct {
                bool rx_no_ack   :1;                    //!< No acknowledgment received for Rx with variable length packets
                bool packet_sent :1;                    //!< Packet sent, only relevant in Tx mode
            } tx_rx_status;                             //!< The Tx/Rx status Byte
            uint8_t sync_addr_status :3;                //!< The id of the correlator who found the packet
        } lora;
        struct {
            int8_t rssi_avg;                            //!< The averaged RSSI
            int8_t rssi_sync;                           //!< The RSSI of the last packet
            struct {
                bool sync_error            :1;          //!< SyncWord error on last packet
                bool length_error          :1;          //!< Length error on last packet
                bool crc_error             :1;          //!< CRC error on last packet
                bool abort_error           :1;          //!< Abort error on last packet
                bool header_received       :1;          //!< Header received on last packet
                bool packet_received       :1;          //!< Packet received
                bool packet_controler_busy :1;          //!< Packet controller busy
            } error_status;                             //!< The error status Byte
            struct {
                uint8_t rx_pid   :2;                    //!< PID of the Rx
                bool rx_no_ack   :1;                    //!< No acknowledgment received for Rx with variable length packets
                bool rx_pid_err  :1;                    //!< Received PID error
                bool packet_sent :1;                    //!< Packet sent, only relevant in Tx mode
            } tx_rx_status;                             //!< The Tx/Rx status Byte
            uint8_t sync_addr_status :3;                //!< The id of the correlator who found the packet
        } flrc;
        struct {
            int8_t rssi_avg;                            //!< The averaged RSSI
            int8_t rssi_sync;                           //!< The RSSI of the last packet
            struct {
                bool sync_error            :1;          //!< SyncWord error on last packet
                bool length_error          :1;          //!< Length error on last packet
                bool crc_error             :1;          //!< CRC error on last packet
                bool abort_error           :1;          //!< Abort error on last packet
                bool header_received       :1;          //!< Header received on last packet
                bool packet_received       :1;          //!< Packet received
                bool packet_controler_busy :1;          //!< Packet controller busy
            } error_status;                             //!< The error status Byte
            struct {
                bool packet_sent :1;                    //!< Packet sent, only relevant in Tx mode
            } tx_rx_status;                             //!< The Tx/Rx status Byte
            uint8_t sync_addr_status :3;                //!< The id of the correlator who found the packet
        } ble;
    } params;
} sx128x_packet_status_t;

/**
 * @brief Represents the Rx internal counters values when GFSK or LORA packet type is used
 */
typedef struct {
    sx128x_radio_packet_types_t packet_type;            //!< Packet to which the packet status are referring to.
    union {
        struct {
            uint16_t packet_received;                   //!< Number of received packets
            uint16_t crc_error;                         //!< Number of CRC errors
            uint16_t length_error;                      //!< Number of length errors
            uint16_t syncword_error;                    //!< Number of sync-word errors
        } gfsk;
        struct {
            uint16_t packet_received;                   //!< Number of received packets
            uint16_t crc_error;                         //!< Number of CRC errors
            uint16_t header_valid;                      //!< Number of valid headers
        } lora;
    } params;
} sx128x_rx_counter_t;

/**
 * @brief Represents a calibration configuration
 */
typedef struct
{
    uint8_t rc64k_enable     : 1;                       /**< Calibrate RC64K clock */
    uint8_t rc13m_enable     : 1;                       /**< Calibrate RC13M clock */
    uint8_t pll_enable       : 1;                       /**< Calibrate PLL         */
    uint8_t adc_pulse_enable : 1;                       /**< Calibrate ADC Pulse   */
    uint8_t adc_bulkn_enable : 1;                       /**< Calibrate ADC bulkN   */
    uint8_t adc_bulkp_enable : 1;                       /**< Calibrate ADC bulkP   */
} sx128x_calibration_params_t;

/**
 * @brief Represents a sleep mode configuration
 */
typedef struct {
    uint8_t wakeup_rtc               : 1;               /**< Get out of sleep mode if wakeup signal received from RTC */
    uint8_t instruction_ram_retention : 1;              /**< InstructionRam is conserved during sleep */
    uint8_t data_buffer_retention     : 1;              /**< Data buffer is conserved during sleep */
    uint8_t data_ram_retention        : 1;              /**< Data ram is conserved during sleep */
} sx128x_sleep_params_t;

/**
 * @brief Returns the value of LoRa bandwidth from driver's value
 *
 * The value is returned in Hz so that it can be represented as an integer
 * type. Most computation should be done as integer to reduce floating
 * point related errors.
 *
 * @retval bw_value            The value of the current bandwidth in Hz
 */
int32_t sx1280_get_lora_bandwidth(void);

/**
 * @brief Returns the corrected raw value of ranging 
 *
 * @retval correction              Corrected ranging raw value 
 */
int32_t sx1280_get_ranging_correction_per_sf_bw_gain(const sx128x_radio_lora_spreading_factors_t sf, const sx128x_radio_lora_bandwidths_t bw, const int8_t gain);

/**
 * @brief Returns the short range corrected distance
 *
 * @retval Corrected Distance              corrected ditance
 */
double sx1280_compute_ranging_correction_polynome(const sx128x_radio_lora_spreading_factors_t sf, const sx128x_radio_lora_bandwidths_t bw, const double median);

/**
 * @brief DIOs interrupt callback
 *
 * @remark Called to handle all 3 DIOs pins
 * 
 * @param[in]   dev       Pointer to device descriptor
 */
void sx1280_on_dio_irq(void *arg);

/**
 * @brief Set the role of the radio during ranging operations
 *
 * @param[in]  role          Role of the radio
 */
void sx1280_set_ranging_role(const sx128x_t *dev, sx128x_radio_ranging_roles_t role);

/**
 * @brief   Initializes the radio driver
 *
 * @param[in/out] dev           Pointer to device descriptor
 * @param[in]     params        Structure containing the hardware settings
 * @param[in]     callbacks     Structure containing the driver callback functions
 */
void sx1280_init(sx128x_t *dev, const sx128x_params_t *params, sx128x_radio_callbacks_t *callbacks);

/**
 * @brief Set the driver in polling mode.
 *
 * In polling mode the application is responsible to call sx1820_process_irqs( ) to
 * execute callbacks functions.
 * The default mode is Interrupt Mode.
 * @code
 * // Initializations and callbacks declaration/definition
 * FIXME: radio = SX1280(mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks);
 * sx1280_init();
 * sx1280_set_polling_mode();
 *
 * while(true) {
 *                             //     IRQ processing is automatically done
 *     sx1280_process_irqs();  // <-- here, as well as callback functions
 *                             //     calls
 *     // Do some applicative work
 * }
 * @endcode
 *
 * @see sx1280_set_interrupt_mode
 */
void sx1280_set_polling_mode(void);

/**
 * @brief Set the driver in interrupt mode.
 *
 * In interrupt mode, the driver communicate with the radio during the
 * interruption by direct calls to ProcessIrqs( ). The main advantage is
 * the possibility to have low power application architecture.
 * This is the default mode.
 * 
 * @code
 * // Initializations and callbacks declaration/definition
 * FIXME: radio = SX1280(mosi, miso, sclk, nss, busy, int1, int2, int3, rst, &callbacks);
 * sx1280_init();
 * sx1280_set_interrupt_mode();   // Optional. Driver default behavior
 *
 * while(true) {
 *     // Do some applicative work
 * }
 * @endcode
 *
 * @see sx1280_set_polling_mode
 */
void sx1280_set_interrupt_mode(void);

/**
 * @brief Initializes the radio registers to the recommended default values
 * 
 * @param[in]  dev   Pointer to device descriptor
 */
void sx1280_set_registers_default(const sx128x_t *dev);

/**
 * @brief Returns the current device firmware version
 * 
 * @param[in]   dev       Pointer to device descriptor
 *
 * @retval      version   Firmware version
 */
uint16_t sx1280_get_firmware_version(const sx128x_t *dev);

/**
 * @brief Gets the current Operation Mode of the Radio
 *
 * @retval      sx128x_radio_operating_modes_t last operating mode
 */
sx128x_radio_operating_modes_t sx1280_get_opmode(void);

/**
 * @brief Gets the current radio status
 *
 * @param[in]   dev         Pointer to device descriptor
 * 
 * @retval      status      Radio status
 */
sx128x_radio_status_t sx1280_get_status(const sx128x_t *dev);

/**
 * @brief Sets the radio in sleep mode
 *
 * @param[in]  dev              Pointer to device descriptor
 * @param[in]  sleep_config     The sleep configuration describing data
 *                              retention and RTC wake-up
 */
void sx1280_set_sleep(const sx128x_t *dev, sx128x_sleep_params_t sleep_config);

/**
 * @brief Sets the radio in configuration mode
 *
 * @param[in]  dev              Pointer to device descriptor 
 * @param[in]  standby_config   The standby mode to put the radio into
 */
void sx1280_set_standby(const sx128x_t *dev, sx128x_radio_standby_modes_t standby_config);

/**
 * @brief Sets the radio in FS mode
 * 
 * @param[in]  dev      Pointer to device descriptor
 */
void sx1280_set_fs(const sx128x_t *dev);

/**
 * @brief Sets the radio in transmission mode
 *
 * @param[in]  dev          Pointer to device descriptor
 * @param[in]  timeout      Structure describing the transmission timeout value
 */
void sx1280_set_tx(const sx128x_t *dev, sx128x_tick_time_t timeout);

/**
 * @brief Sets the radio in reception mode
 *
 * @param[in]  dev          Pointer to device descriptor
 * @param[in]  timeout      Structure describing the reception timeout value
 */
void sx1280_set_rx(const sx128x_t *dev, sx128x_tick_time_t timeout);

/**
 * @brief Sets the Rx duty cycle management parameters
 *
 * @param[in]  dev                  Pionter to devise descriptor
 * @param[in]  step                 Structure discribing ...
 * @param[in]  nb_step_rx           Structure describing reception timeout value
 * @param[in]  rx_nb_step_sleep     Structure describing sleep timeout value
 */

void sx1280_set_rx_duty_cycle(const sx128x_t *dev, sx128x_radio_tick_sizes_t step, uint16_t nb_step_rx, uint16_t rx_nb_step_sleep);

/**
 * @brief Sets the radio in CAD mode
 *
 * @param[in]  dev      pointer to device descriptor
 * 
 * @see sx1280_set_cad_params
 */
void sx1280_set_cad(const sx128x_t *dev);

/**
 * @brief Sets the radio in continuous wave transmission mode
 * 
 * @param[in]  dev      Pointer to device descriptor
 */
void sx1280_set_tx_continuous_wave(const sx128x_t *dev);

/**
 * @brief Sets the radio in continuous preamble transmission mode
 * 
 * @param[in]  dev      Pointer to device descriptor
 */
void sx1280_set_tx_continuous_preamble(const sx128x_t *dev);

/**
 * @brief Sets the radio for the given protocol
 *
 * @param[in]  dev              Pointer to device descriptor
 * @param[in]  pkt_type         [SX128X_PACKET_TYPE_GFSK, SX128X_PACKET_TYPE_LORA,
 *                               SX128X_PACKET_TYPE_RANGING, SX128X_PACKET_TYPE_FLRC,
 *                               SX128X_PACKET_TYPE_BLE]
 *
 * @remark This method has to be called before set_rf_frequency,
 *         set_modulation_params and set_packet_params
 */
void sx1280_set_packet_type(const sx128x_t *dev, sx128x_radio_packet_types_t pkt_type);

/**
 * @brief Gets the current radio protocol
 *
 * @retval  pkt_type        [SX128X_PACKET_TYPE_GFSK, SX128X_PACKET_TYPE_LORA,
 *                           SX128X_PACKET_TYPE_RANGING, SX128X_PACKET_TYPE_FLRC,
 *                           SX128X_PACKET_TYPE_BLE, SX128X_PACKET_TYPE_NONE]
 */
sx128x_radio_packet_types_t sx1280_get_packet_type(void);

/**
 * @brief Sets the RF frequency
 *
 * @param[in]  dev          Pointer to device descriptor
 * @param[in]  frequency    RF frequency [Hz]
 */
void sx1280_set_rf_frequency(const sx128x_t *dev, uint32_t frequency);

/**
 * @brief Sets the transmission parameters
 *
 * @param[in]  dev          Pointer to device descriptor
 * @param[in]  power        RF output power [-18..13] dBm
 * @param[in]  ramp_time    Transmission ramp up time
 */
void sx1280_set_tx_params(const sx128x_t *dev, int8_t power, sx128x_radio_ramp_times_t ramp_time);

/**
 * @brief Sets the number of symbols to be used for Channel Activity
 *        Detection operation
 *
 * @param[in]  dev              Pointer to device descriptor
 * @param[in]  cad_symbol_num   The number of symbol to use for Channel Activity
 *                              Detection operations [LORA_CAD_01_SYMBOL, LORA_CAD_02_SYMBOL,
 *                              LORA_CAD_04_SYMBOL, LORA_CAD_08_SYMBOL, LORA_CAD_16_SYMBOL]
 */
void sx1280_set_cad_params(const sx128x_t *dev, sx128x_radio_lora_cad_symbols_t cad_symbol_num);

/**
 * @brief Sets the data buffer base address for transmission and reception
 *
 * @param[in]  dev              Pointer to device descriptor
 * @param[in]  tx_base_address  Transmission base address
 * @param[in]  rx_base_address  Reception base address
 */
void sx1280_set_buffer_base_addresses(const sx128x_t *dev, uint8_t tx_base_address, uint8_t rx_base_address);

/**
 * @brief Set the modulation parameters
 *
 * @param[in]  dev                  Pointer to device descriptor
 * @param[in]  modulation_params    A structure describing the modulation parameters
 */
void sx1280_set_modulation_params(const sx128x_t *dev, sx128x_modulation_params_t *modulation_params);

/**
 * @brief Sets the packet parameters
 *
 * @param[in]  dev                  Pointer to device descriptor
 * @param[in]  packet_params        A structure describing the packet parameters
 */
void sx1280_set_packet_params(const sx128x_t *dev, sx128x_packet_params_t *packet_params);

/**
 * @brief Gets the last received packet buffer status
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[out] payload_length           Last received packet payload length
 * @param[out] rx_start_buffer_pointer  Last received packet buffer address pointer
 */
void sx1280_get_rx_buffer_status(const sx128x_t *dev, uint8_t *payload_length, uint8_t *rx_start_buffer_pointer);

/**
 * @brief Gets the last received packet payload length
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[out] pkt_status               A structure of packet status
 */
void sx1280_get_packet_status(const sx128x_t *dev, sx128x_packet_status_t *pkt_status);

/**
 * @brief Returns the instantaneous RSSI value for the last packet received
 *
 * @param[in]  dev                      Pointer to device descriptor
 * 
 * @retval     rssiInst                 Instantaneous RSSI
 */
int8_t sx1280_get_rssi_inst(const sx128x_t *dev);

/**
 * @brief   Sets the IRQ mask and DIO masks
 *
 * 
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  irq_mask                 General IRQ mask
 * @param[in]  dio1_mask                DIO1 mask
 * @param[in]  dio2_mask                DIO2 mask
 * @param[in]  dio3_mask                DIO3 mask
 */
void sx1280_set_dio_irq_params(const sx128x_t *dev, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);

/**
 * @brief Returns the current IRQ status
 *
 * @param[in]  dev                      Pointer to device descriptor
 * 
 * @retval     irq_status               IRQ status
 */
uint16_t sx1280_get_irq_status(const sx128x_t *dev);

/**
 * @brief Clears the IRQs
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  irq                      IRQ(s) to be cleared
 */
void sx1280_clear_irq_status(const sx128x_t *dev, uint16_t irq);

/**
 * @brief Calibrates the given radio block
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  calib_param              The description of blocks to be calibrated
 */
void sx1280_calibrate(const sx128x_t *dev, sx128x_calibration_params_t calib_param);

/**
 * @brief Sets the power regulators operating mode
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  mode                     [0: LDO, 1:DC_DC]
 */
void sx1280_set_regulator_mode(const sx128x_t *dev, sx128x_radio_regulator_modes_t mode);

/**
 * @brief Saves the current selected modem configuration into data RAM
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_set_save_context(const sx128x_t *dev);

/**
 * @brief Sets the chip to automatically send a packet after the end of a packet reception
 *
 * @remark The offset is automatically compensated inside the function
 *
 * 
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  time                     The delay in us after which a Tx is done
 */
void sx1280_set_auto_tx(const sx128x_t *dev, uint16_t time);

/**
 * @brief Stop the chip from automatically sending a packet after the end of a packet reception
 * if previously activated with sx1280_set_auto_tx command
 *
 * @see sx1280_set_auto_tx
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_stop_auto_tx(const sx128x_t *dev);

/**
 * @brief Sets the chip to automatically receive a packet after the end of a packet transmission
 *
 * @remark The offset is automatically compensated inside the function
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  enable                   [0: Disable, 1: Enable] ???The delay in us after which a Rx is done
 */
void sx1280_set_auto_fs(const sx128x_t *dev, uint8_t enable);

/**
 * @brief Enables or disables long preamble detection mode
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  enable                   [0: Disable, 1: Enable]
 */
void sx1280_set_long_preamble(const sx128x_t *dev, uint8_t enable);

/**
 * @brief Saves the payload to be send in the radio buffer
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  buffer                   A pointer to the payload
 * @param[in]  size                     The size of the payload
 */
void sx1280_set_payload(const sx128x_t *dev, uint8_t *buffer, uint8_t size);

/**
 * @brief Reads the payload received. If the received payload is longer
 * than maxSize, then the method returns 1 and do not set size and payload.
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[out] buffer                   A pointer to a buffer into which the payload will be copied
 * @param[out] size                     A pointer to the size of the payload received
 * @param[in]  max_size                 The maximal size allowed to copy into the buffer
 */
uint8_t sx1280_get_payload(const sx128x_t *dev, uint8_t *buffer, uint8_t *size , uint8_t max_size);

/**
 * @brief Sends a payload
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  payload                  A pointer to the payload to send
 * @param[in]  size                     The size of the payload to send
 * @param[in]  timeout                  The timeout for Tx operation
 */
void sx1280_send_payload(const sx128x_t *dev, uint8_t *payload, uint8_t size, sx128x_tick_time_t timeout);

/**
 * @brief Sets the Sync Word given by index used in GFSK, FLRC and BLE protocols
 *
 * @remark 5th byte isn't used in FLRC and BLE protocols
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  sync_word_idx            Index of SyncWord to be set [1..3]
 * @param[in]  sync_word                SyncWord bytes ( 5 bytes )
 *
 * @retval     status                   [0: OK, 1: NOK]
 */
uint8_t sx1280_set_sync_word(const sx128x_t *dev, uint8_t sync_word_idx, uint8_t *sync_word);

/**
 * @brief Defines how many error bits are tolerated in sync word detection
 *
 * 
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  error_bits               Number of error bits supported to validate the Sync word detection
 *                                      (default is 4 bit, minimum is 1 bit)
 */
void sx1280_set_sync_word_error_tolerance(const sx128x_t *dev, uint8_t error_bits);

/**
 * @brief Sets the Initial value for the LFSR used for the CRC calculation
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  seed                     Initial LFSR value ( 4 bytes )
 */
void sx1280_set_crc_seed(const sx128x_t *dev, uint16_t seed);

/**
 * @brief Set the Access Address field of BLE packet
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  access_address           The access address to be used for next BLE packet sent
 *
 * @see sx1280_set_ble_advertizer_access_address
 */
void sx1280_set_ble_access_address(const sx128x_t *dev, uint32_t access_address);

/**
 * @brief Set the Access Address for Advertizer BLE packets
 *
 * All advertizer BLE packets must use a particular value for Access
 * Address field. This method sets it.
 *
 * @see sx1280_set_ble_access_address
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_set_ble_advertizer_access_address(const sx128x_t *dev);

/**
 * @brief Sets the seed used for the CRC calculation
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  polynomial               The seed value
 */
void sx1280_set_crc_polynomial(const sx128x_t *dev, uint16_t polynomial);

/**
 * @brief Sets the Initial value of the LFSR used for the whitening in GFSK, FLRC and BLE protocols
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  seed                     Initial LFSR value
 */
void sx1280_set_whitening_seed(const sx128x_t *dev, uint8_t seed);

/**
 * @brief Enable manual gain and disable AGC
 *
 * @see sx1280_set_manual_gain_value, sx1280_disable_manual_gain
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_enable_manual_gain(const sx128x_t *dev);

/**
 * @brief Disable the manual gain control and enable AGC
 *
 * @see sx1280_enable_manual_gain
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_disable_manual_gain(const sx128x_t *dev);

/**
 * @brief Set the gain for LNA
 *
 * SX1280EnableManualGain must be call before using this function
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  gain                     The value of gain to set, refer to datasheet for value meaning
 *
 * @see sx1280_enable_manual_gain, sx1280_disable_manual_gain
 */
void sx1280_set_manual_gain_value(const sx128x_t *dev, uint8_t gain);

/**
 * @brief Configure the LNA regime of operation
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  lna_setting              The LNA setting. Possible values are
 *                                      SX128X_LNA_LOW_POWER_MODE and
 *                                      SX128X_LNA_HIGH_SENSITIVITY_MODE
 */
void sx1280_set_lna_gain_setting(const sx128x_t *dev, const sx128x_radio_lna_settings_t lna_setting);

/**
 * @brief Sets the number of bits used to check that ranging request match ranging ID
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  length                   [0: 8 bits, 1: 16 bits,
 *                                       2: 24 bits, 3: 32 bits]
 */
void sx1280_set_ranging_id_length(const sx128x_t *dev, sx128x_radio_ranging_id_check_lengths_t length);

/**
 * @brief Sets ranging device id
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  address                  Device address
 */
void sx1280_set_device_ranging_address(const sx128x_t *dev, uint32_t address);

/**
 * @brief Sets the device id to ping in a ranging request
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  address                  Address of the device to ping
 */
void sx1280_set_ranging_request_address(const sx128x_t *dev, uint32_t address);

/**
 * @brief Return the ranging result value
 *
 * 
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  result_type              Specifies the type of result.
 *                                      [0: RAW, 1: Averaged,
 *                                       2: De-biased, 3:Filtered]
 *
 * @retval     ranging                  The ranging measure filtered according to resultType [m]
 */
int32_t sx1280_get_ranging_result(const sx128x_t *dev, sx128x_radio_ranging_result_types_t result_type);

/**
 * @brief Sets the standard processing delay between Master and Slave
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  cal                      RxTx delay offset for correcting ranging bias.
 *
 * The calibration value reflects the group delay of the radio front end and
 * must be re-performed for each new SX1280 PCB design. The value is obtained
 * empirically by either conducted measurement in a known electrical length
 * coaxial RF cable (where the design is connectorised) or by radiated
 * measurement, at a known distance, where an antenna is present.
 * The result of the calibration process is that the SX1280 ranging result
 * accurately reflects the physical range, the calibration procedure therefore
 * removes the average timing error from the time-of-flight measurement for a
 * given design.
 *
 * The values are Spreading Factor dependents, and depend also of the board
 * design. Some typical values are provided in the next table.
 *
 * Spreading Factor | Calibration Value
 * ---------------- | -----------------
 *   SF5            |  12200
 *   SF6            |  12200
 *   SF7            |  12400
 *   SF8            |  12650
 *   SF9            |  12940
 *   SF10           |  13000
 *   SF11           |  13060
 *   SF12           |  13120
 */
void sx1280_set_ranging_calibration(const sx128x_t *dev, uint16_t cal);

/**
 * @brief Return the last ranging result power indicator
 *
 * The value returned is not an absolute power measurement. It is
 * a relative power measurement.
 *
 * @param[in]  dev                      Pointer to device descriptor
 * 
 * @retval     delta_threshold          A relative power indicator
 */
uint8_t sx1280_get_ranging_power_delta_threshold_indicator(const sx128x_t *dev);

/**
 * @brief Clears the ranging filter
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_ranging_clear_filter_result(const sx128x_t *dev);

/**
 * @brief Set the number of samples considered in the built-in filter
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  num                      The number of samples to use built-in filter
 *                                      [8..255]
 *
 * @remark Value inferior to 8 will be silently set to 8
 */
void sx1280_ranging_set_filter_num_samples(const sx128x_t *dev, uint8_t num);

/**
 * @brief Return the Estimated Frequency Error in LORA and RANGING operations
 *
 * @param[in]  dev                      Pointer to device descriptor
 * 
 * @retval     efe                      The estimated frequency error [Hz]
 */
int32_t sx1280_get_frequency_error(const sx128x_t *dev);

/**
 * @brief Process the analysis of radio IRQs and calls callback functions
 *        depending on radio state
 * 
 * @param[in]  dev                      Pointer to device descriptor
 */
void sx1280_process_irqs(const sx128x_t *dev);

/**
 * @brief Parses 1 HEX file line and writes the content to the instruction memory
 *
 * @param[in]  dev                      Pointer to device descriptor
 * @param[in]  line                     HEX file line string
 *
 * @retval     status                   [0: ERROR, 1:OK]
 */
int8_t sx1280_parse_hex_file_line(const sx128x_t *dev, char *line);

/**
 * @brief Gets individual fields for the given HEX file line
 *
 * @param[in]  line          HEX file line string
 * @param[out] bytes         Bytes array to be written to the instruction memory
 * @param[out] addr          Instruction memory address where to write the bytes array
 * @param[out] num           Number of bytes in Bytes array
 * @param[out] code          HEX file line type [0: instruction, 1: end of file, 2: begin of file]
 *
 * @retval     status        [0: ERROR, 1:OK]
 */
int8_t sx1280_get_hex_file_line_fields(char *line, uint8_t *bytes, uint16_t *addr, uint16_t *num, uint8_t *code);

#ifdef __cplusplus
}
#endif

#endif /* _SX128X_H_ */
/** @} */