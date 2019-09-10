/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 *
 * @file
 * @brief       SX128X registers
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 * @}
 */

#ifndef _SX128X_REGISTERS_H_
#define _SX128X_REGISTERS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name   SX128X LoRa modem internal registers addresses
 * @{
 */

/**
 * @brief The address of the register holding the firmware version MSB
 */
#define SX128X_REG_LR_FW_VER_MSB                        (0x0153)

/**
 * @brief The address of the register holding the first byte defining the CRC seed
 *
 * @remark Only used for packet types GFSK and Flrc
 */
#define SX128X_REG_LR_CRCSEEDBASEADDR                   (0x09C8)

/**
 * @brief The address of the register holding the first byte defining the CRC polynomial
 *
 * @remark Only used for packet types GFSK and Flrc
 */
#define SX128X_REG_LR_CRCPOLYBASEADDR                   (0x09C6)

/**
 * @brief The address of the register holding the first byte defining the whitening seed
 *
 * @remark Only used for packet types GFSK, FLRC and BLE
 */
#define SX128X_REG_LR_WHITSEEDBASEADDR                  (0x09C5)

/**
 * @brief The address of the register holding the ranging id check length
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGIDCHECKLENGTH              (0x0931)

/**
 * @brief The address of the register holding the device ranging id
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_DEVICERANGINGADDR                 (0x0916)

/**
 * @brief The address of the register holding the device ranging id
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_REQUESTRANGINGADDR                (0x0912)

/**
 * @brief The address of the register holding ranging results configuration
 * and the corresponding mask
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGRESULTCONFIG               (0x0924)
#define SX128X_MASK_RANGINGMUXSEL                       (0xCF)

/**
 * @brief The address of the register holding the first byte of ranging results
 * 
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGRESULTBASEADDR             (0x0961)

/**
 * @brief The address of the register allowing to read ranging results
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGRESULTSFREEZE              (0x097F)

/**
 * @brief The address of the register holding the first byte of ranging calibration
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGRERXTXDELAYCAL             (0x092C)

/**
 * @brief The address of the register holding the ranging filter window size
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGFILTERWINDOWSIZE           (0x091E)

/**
 * @brief The address of the register to reset for clearing ranging filter
 *
 * @remark Only used for packet type Ranging
 */
#define SX128X_REG_LR_RANGINGRESULTCLEARREG             (0x0923)

#define SX128X_REG_RANGING_RSSI                         (0x0964)

/**
 * @brief The default number of samples considered in built-in ranging filter
 */
#define SX128X_DEFAULT_RANGING_FILTER_SIZE              (127)

/**
 * @brief The address of the register holding LORA packet parameters
 */
#define SX128X_REG_LR_PACKETPARAMS                      (0x903)

/**
 * @brief The address of the register holding payload length
 *
 * @remark Do NOT try to read it directly. Use GetRxBuffer( ) instead.
 */
#define SX128X_REG_LR_PAYLOADLENGTH                     (0x901)

/**
 * @brief The address of the instruction RAM and its size
 */
#define SX128X_IRAM_START_ADDRESS                       (0x8000)
#define SX128X_IRAM_SIZE                                (0x4000)

/**
 * @brief The addresses of the registers holding SyncWords values
 *
 * @remark The addresses depends on the Packet Type in use, and not all
 *         SyncWords are available for every Packet Type
 */
#define SX128X_REG_LR_SYNCWORDBASEADDRESS1              (0x09CE)
#define SX128X_REG_LR_SYNCWORDBASEADDRESS2              (0x09D3)
#define SX128X_REG_LR_SYNCWORDBASEADDRESS3              (0x09D8)

/**
 * @brief The MSB address and mask used to read the estimated frequency
 * error
 */
#define SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB     (0x0954)
#define SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK    (0x0FFFFF)

/**
 * @brief Defines how many bit errors are tolerated in sync word detection
 */
#define SX128X_REG_LR_SYNCWORDTOLERANCE                 (0x09CD)

/**
 * @brief Register for MSB Access Address (BLE)
 */
#define SX128X_REG_LR_BLE_ACCESS_ADDRESS                (0x09CF)
#define SX128X_BLE_ADVERTIZER_ACCESS_ADDRESS            (0x8E89BED6)

/**
 * @brief Register address and mask for LNA regime selection
 */
#define SX128X_REG_LNA_REGIME                           (0x0891)
#define SX128X_MASK_LNA_REGIME                          (0xC0)

/**
 * @brief Register and mask enabling manual gain control
 */
#define SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL           (0x089F)
#define SX128X_MASK_MANUAL_GAIN_CONTROL                 (0x80)

/**
 * @brief Register and mask controlling demodulation detection
 */
#define SX128X_REG_DEMOD_DETECTION                      (0x0895)
#define SX128X_MASK_DEMOD_DETECTION                     (0xFE)

/**
 * @brief Register and mask setting manual gain value
 */
#define SX128X_REG_MANUAL_GAIN_VALUE                    (0x089E)
#define SX128X_MASK_MANUAL_GAIN_VALUE                   (0xF0)

/**
 * @brief Selector values to configure LNA regime
 */
typedef enum{
    SX128X_LNA_LOW_POWER_MODE,              /**< Allow maximum efficiency of sx1280 (default) */
    SX128X_LNA_HIGH_SENSITIVITY_MODE,       /**< Allow to use highest three steps of LNA gain and increase current consumption */
}RadioLnaSettings_t;

/**
 * @brief Represents the states of the radio
 */
typedef enum
{
    SX128X_RF_IDLE         = 0x00,          /**< The radio is idle */
    SX128X_RF_RX_RUNNING,                   /**< The radio is in reception state */
    SX128X_RF_TX_RUNNING,                   /**< The radio is in transmission state */
    SX128X_RF_CAD,                          /**< The radio is doing channel activity detection */
}RadioStates_t;

/**
 * @brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    SX128X_MODE_SLEEP        = 0x00,        /**< The radio is in sleep mode */
    SX128X_MODE_STDBY_RC,                   /**< The radio is in standby mode with RC oscillator */
    SX128X_MODE_STDBY_XOSC,                 /**< The radio is in standby mode with XOSC oscillator */
    SX128X_MODE_FS,                         /**< The radio is in frequency synthesis mode */
    SX128X_MODE_TX,                         /**< The radio is in transmit mode */
    SX128X_MODE_RX,                         /**< The radio is in receive mode */
    SX128X_MODE_CAD                         /**< The radio is in channel activity detection mode */
}RadioOperatingModes_t;

/**
 * @brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
    SX128X_STDBY_RC   = 0x00,
    SX128X_STDBY_XOSC = 0x01,
}RadioStandbyModes_t;

/**
 * @brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
    SX128X_USE_LDO  = 0x00,                 /**< Use LDO (default value) */
    SX128X_USE_DCDC = 0x01,                 /**< Use DCDC */
}RadioRegulatorModes_t;

/**
 * @brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
    SX128X_PACKET_TYPE_GFSK     = 0x00,
    SX128X_PACKET_TYPE_LORA,
    SX128X_PACKET_TYPE_RANGING,
    SX128X_PACKET_TYPE_FLRC,
    SX128X_PACKET_TYPE_BLE,
    SX128X_PACKET_TYPE_NONE     = 0x0F,
}RadioPacketTypes_t;

/**
 * @brief Represents the ramping time for power amplifier
 */
typedef enum
{
    SX128X_RADIO_RAMP_02_US = 0x00,
    SX128X_RADIO_RAMP_04_US = 0x20,
    SX128X_RADIO_RAMP_06_US = 0x40,
    SX128X_RADIO_RAMP_08_US = 0x60,
    SX128X_RADIO_RAMP_10_US = 0x80,
    SX128X_RADIO_RAMP_12_US = 0xA0,
    SX128X_RADIO_RAMP_16_US = 0xC0,
    SX128X_RADIO_RAMP_20_US = 0xE0,
}RadioRampTimes_t;

/**
 * @brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
    SX128X_LORA_CAD_01_SYMBOL = 0x00,
    SX128X_LORA_CAD_02_SYMBOL = 0x20,
    SX128X_LORA_CAD_04_SYMBOL = 0x40,
    SX128X_LORA_CAD_08_SYMBOL = 0x60,
    SX128X_LORA_CAD_16_SYMBOL = 0x80,
}RadioLoRaCadSymbols_t;

/**
 * @brief Represents the possible combinations of bitrate and bandwidth for
 *        GFSK and BLE packet types
 *
 * The bitrate is expressed in Mb/s and the bandwidth in MHz
 */
typedef enum
{
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
}RadioGfskBleBitrates_t;

/**
 * @brief Represents the modulation index used in GFSK and BLE packet
 *        types
 */
typedef enum
{
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
}RadioGfskBleModIndexes_t;

/**
 * @brief Represents the possible combination of bitrate and bandwidth for FLRC
 *        packet type
 *
 * The bitrate is in Mb/s and the bitrate in MHz
 */
typedef enum
{
    SX128X_FLRC_BR_2_600_BW_2_4 = 0x04,
    SX128X_FLRC_BR_2_080_BW_2_4 = 0x28,
    SX128X_FLRC_BR_1_300_BW_1_2 = 0x45,
    SX128X_FLRC_BR_1_040_BW_1_2 = 0x69,
    SX128X_FLRC_BR_0_650_BW_0_6 = 0x86,
    SX128X_FLRC_BR_0_520_BW_0_6 = 0xAA,
    SX128X_FLRC_BR_0_325_BW_0_3 = 0xC7,
    SX128X_FLRC_BR_0_260_BW_0_3 = 0xEB,
}RadioFlrcBitrates_t;

/**
 * @brief Represents the possible values for coding rate parameter in FLRC
 *        packet type
 */
typedef enum
{
    SX128X_FLRC_CR_1_2 = 0x00,
    SX128X_FLRC_CR_3_4 = 0x02,
    SX128X_FLRC_CR_1_0 = 0x04,
}RadioFlrcCodingRates_t;

/**
 * @brief Represents the modulation shaping parameter for GFSK, FLRC and BLE
 *        packet types
 */
typedef enum
{
    SX128X_RADIO_MOD_SHAPING_BT_OFF = 0x00,         /**< No filtering */
    SX128X_RADIO_MOD_SHAPING_BT_1_0 = 0x10,
    SX128X_RADIO_MOD_SHAPING_BT_0_5 = 0x20,
}RadioModShapings_t;

/**
 * @brief Represents the possible spreading factor values in LORA packet types
 */
typedef enum
{
    SX128X_LORA_SF5  = 0x50,
    SX128X_LORA_SF6  = 0x60,
    SX128X_LORA_SF7  = 0x70,
    SX128X_LORA_SF8  = 0x80,
    SX128X_LORA_SF9  = 0x90,
    SX128X_LORA_SF10 = 0xA0,
    SX128X_LORA_SF11 = 0xB0,
    SX128X_LORA_SF12 = 0xC0,
}RadioLoRaSpreadingFactors_t;

/**
 * @brief Represents the bandwidth values for LORA packet type
 */
typedef enum
{
    SX128X_LORA_BW_0200 = 0x34,
    SX128X_LORA_BW_0400 = 0x26,
    SX128X_LORA_BW_0800 = 0x18,
    SX128X_LORA_BW_1600 = 0x0A,
}RadioLoRaBandwidths_t;

/**
 * @brief Represents the coding rate values for LORA packet type
 */
typedef enum
{
    SX128X_LORA_CR_4_5    = 0x01,
    SX128X_LORA_CR_4_6    = 0x02,
    SX128X_LORA_CR_4_7    = 0x03,
    SX128X_LORA_CR_4_8    = 0x04,
    SX128X_LORA_CR_LI_4_5 = 0x05,
    SX128X_LORA_CR_LI_4_6 = 0x06,
    SX128X_LORA_CR_LI_4_7 = 0x07,
}RadioLoRaCodingRates_t;

/**
 * @brief Represents the preamble length values for GFSK and FLRC packet
 *        types
 */
typedef enum
{
    SX128X_PREAMBLE_LENGTH_04_BITS = 0x00,              /**< Preamble length: 04 bits */
    SX128X_PREAMBLE_LENGTH_08_BITS = 0x10,              /**< Preamble length: 08 bits */
    SX128X_PREAMBLE_LENGTH_12_BITS = 0x20,              /**< Preamble length: 12 bits */
    SX128X_PREAMBLE_LENGTH_16_BITS = 0x30,              /**< Preamble length: 16 bits */
    SX128X_PREAMBLE_LENGTH_20_BITS = 0x40,              /**< Preamble length: 20 bits */
    SX128X_PREAMBLE_LENGTH_24_BITS = 0x50,              /**< Preamble length: 24 bits */
    SX128X_PREAMBLE_LENGTH_28_BITS = 0x60,              /**< Preamble length: 28 bits */
    SX128X_PREAMBLE_LENGTH_32_BITS = 0x70,              /**< Preamble length: 32 bits */
}RadioPreambleLengths_t;

/**
 * @brief Represents the SyncWord length for FLRC packet type
 */
typedef enum
{
    SX128X_FLRC_NO_SYNCWORD            = 0x00,
    SX128X_FLRC_SYNCWORD_LENGTH_4_BYTE = 0x04,
}RadioFlrcSyncWordLengths_t;

/**
 * @brief The length of sync words for GFSK packet type
 */
typedef enum
{
    SX128X_GFSK_SYNCWORD_LENGTH_1_BYTE = 0x00,          /**< Sync word length: 1 byte  */
    SX128X_GFSK_SYNCWORD_LENGTH_2_BYTE = 0x02,          /**< Sync word length: 2 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_3_BYTE = 0x04,          /**< Sync word length: 3 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_4_BYTE = 0x06,          /**< Sync word length: 4 bytes */
    SX128X_GFSK_SYNCWORD_LENGTH_5_BYTE = 0x08,          /**< Sync word length: 5 bytes */
}RadioSyncWordLengths_t;

/**
 * @brief Represents the possible combinations of SyncWord correlators
 *        activated for GFSK and FLRC packet types
 */
typedef enum
{
    SX128X_RADIO_RX_MATCH_SYNCWORD_OFF   = 0x00,        /**< No correlator turned on, i.e. do not search for SyncWord */
    SX128X_RADIO_RX_MATCH_SYNCWORD_1     = 0x10,
    SX128X_RADIO_RX_MATCH_SYNCWORD_2     = 0x20,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_2   = 0x30,
    SX128X_RADIO_RX_MATCH_SYNCWORD_3     = 0x40,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_3   = 0x50,
    SX128X_RADIO_RX_MATCH_SYNCWORD_2_3   = 0x60,
    SX128X_RADIO_RX_MATCH_SYNCWORD_1_2_3 = 0x70,
}RadioSyncWordRxMatchs_t;

/**
 *  @brief Radio packet length mode for GFSK and FLRC packet types
 */
typedef enum
{
    SX128X_RADIO_PACKET_FIXED_LENGTH    = 0x00,         /**< The packet is known on both sides, no header included in the packet */
    SX128X_RADIO_PACKET_VARIABLE_LENGTH = 0x20,         /**< The packet is on variable size, header included */
}RadioPacketLengthModes_t;

/**
 * @brief Represents the CRC length for GFSK and FLRC packet types
 *
 * \warning Not all configurations are available for both GFSK and FLRC
 *          packet type. Refer to the datasheet for possible configuration.
 */
typedef enum
{
    SX128X_RADIO_CRC_OFF     = 0x00,                    /**< No CRC in use */
    SX128X_RADIO_CRC_1_BYTES = 0x10,
    SX128X_RADIO_CRC_2_BYTES = 0x20,
    SX128X_RADIO_CRC_3_BYTES = 0x30,
}RadioCrcTypes_t;

/**
 * @brief Radio whitening mode activated or deactivated for GFSK, FLRC and
 *        BLE packet types
 */
typedef enum
{
    SX128X_RADIO_WHITENING_ON  = 0x00,
    SX128X_RADIO_WHITENING_OFF = 0x08,
}RadioWhiteningModes_t;

/**
 * @brief Holds the packet length mode of a LORA packet type
 */
typedef enum
{
    SX128X_LORA_PACKET_VARIABLE_LENGTH = 0x00,          /**< The packet is on variable size, header included */
    SX128X_LORA_PACKET_FIXED_LENGTH    = 0x80,          /**< The packet is known on both sides, no header included in the packet */
    SX128X_LORA_PACKET_EXPLICIT        = LORA_PACKET_VARIABLE_LENGTH,
    SX128X_LORA_PACKET_IMPLICIT        = LORA_PACKET_FIXED_LENGTH,
}RadioLoRaPacketLengthsModes_t;

/**
 * @brief Represents the CRC mode for LORA packet type
 */
typedef enum
{
    SX128X_LORA_CRC_ON  = 0x20,                         /**< CRC activated */
    SX128X_LORA_CRC_OFF = 0x00,                         /**< CRC not used */
}RadioLoRaCrcModes_t;

/**
 * @brief Represents the IQ mode for LORA packet type
 */
typedef enum
{
    SX128X_LORA_IQ_NORMAL   = 0x40,
    SX128X_LORA_IQ_INVERTED = 0x00,
}RadioLoRaIQModes_t;

/**
 * @brief Represents the length of the ID to check in ranging operation
 */
typedef enum
{
    SX128X_RANGING_IDCHECK_LENGTH_08_BITS = 0x00,
    SX128X_RANGING_IDCHECK_LENGTH_16_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_24_BITS,
    SX128X_RANGING_IDCHECK_LENGTH_32_BITS,
}RadioRangingIdCheckLengths_t;

/**
 * @brief Represents the result type to be used in ranging operation
 */
typedef enum
{
    SX128X_RANGING_RESULT_RAW      = 0x00,
    SX128X_RANGING_RESULT_AVERAGED = 0x01,
    SX128X_RANGING_RESULT_DEBIASED = 0x02,
    SX128X_RANGING_RESULT_FILTERED = 0x03,
}RadioRangingResultTypes_t;

/**
 * @brief Represents the connection state for BLE packet type
 */
typedef enum
{
    SX128X_BLE_PAYLOAD_LENGTH_MAX_31_BYTES  = 0x00,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_37_BYTES  = 0x20,
    SX128X_BLE_TX_TEST_MODE                 = 0x40,
    SX128X_BLE_PAYLOAD_LENGTH_MAX_255_BYTES = 0x80,
}RadioBleConnectionStates_t;

/**
 * @brief Represents the CRC field length for BLE packet type
 */
typedef enum
{
    SX128X_BLE_CRC_OFF = 0x00,
    SX128X_BLE_CRC_3B  = 0x10,
}RadioBleCrcFields_t;

/**
 * @brief Represents the specific packets to use in BLE packet type
 */
typedef enum
{
    SX128X_BLE_PRBS_9       = 0x00,                     /**< Pseudo Random Binary Sequence based on 9th degree polynomial */
    SX128X_BLE_PRBS_15      = 0x0C,                     /**< Pseudo Random Binary Sequence based on 15th degree polynomial */
    SX128X_BLE_EYELONG_1_0  = 0x04,                     /**< Repeated '11110000' sequence */
    SX128X_BLE_EYELONG_0_1  = 0x18,                     /**< Repeated '00001111' sequence */
    SX128X_BLE_EYESHORT_1_0 = 0x08,                     /**< Repeated '10101010' sequence */
    SX128X_BLE_EYESHORT_0_1 = 0x1C,                     /**< Repeated '01010101' sequence */
    SX128X_BLE_ALL_1        = 0x10,                     /**< Repeated '11111111' sequence */
    SX128X_BLE_ALL_0        = 0x14,                     /**< Repeated '00000000' sequence */
}RadioBlePacketTypes_t;

/**
 * @brief Represents the interruption masks available for the radio
 *
 * @remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
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
}RadioIrqMasks_t;

/**
 * @brief Represents the digital input/output of the radio
 */
typedef enum
{
    SX128X_RADIO_DIO1 = 0x02,
    SX128X_RADIO_DIO2 = 0x04,
    SX128X_RADIO_DIO3 = 0x08,
}RadioDios_t;

/**
 * @brief Represents the tick size available for Rx/Tx timeout operations
 */
typedef enum
{
    SX128X_RADIO_TICK_SIZE_0015_US = 0x00,
    SX128X_RADIO_TICK_SIZE_0062_US = 0x01,
    SX128X_RADIO_TICK_SIZE_1000_US = 0x02,
    SX128X_RADIO_TICK_SIZE_4000_US = 0x03,
}RadioTickSizes_t;

/**
 * @brief Represents the role of the radio during ranging operations
 */
typedef enum
{
    SX128X_RADIO_RANGING_ROLE_SLAVE  = 0x00,
    SX128X_RADIO_RANGING_ROLE_MASTER = 0x01,
}RadioRangingRoles_t;

/**
 * @brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_u
{
    SX128X_RADIO_GET_STATUS               = 0xC0,
    SX128X_RADIO_WRITE_REGISTER           = 0x18,
    SX128X_RADIO_READ_REGISTER            = 0x19,
    SX128X_RADIO_WRITE_BUFFER             = 0x1A,
    SX128X_RADIO_READ_BUFFER              = 0x1B,
    SX128X_RADIO_SET_SLEEP                = 0x84,
    SX128X_RADIO_SET_STANDBY              = 0x80,
    SX128X_RADIO_SET_FS                   = 0xC1,
    SX128X_RADIO_SET_TX                   = 0x83,
    SX128X_RADIO_SET_RX                   = 0x82,
    SX128X_RADIO_SET_RXDUTYCYCLE          = 0x94,
    SX128X_RADIO_SET_CAD                  = 0xC5,
    SX128X_RADIO_SET_TXCONTINUOUSWAVE     = 0xD1,
    SX128X_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
    SX128X_RADIO_SET_PACKETTYPE           = 0x8A,
    SX128X_RADIO_GET_PACKETTYPE           = 0x03,
    SX128X_RADIO_SET_RFFREQUENCY          = 0x86,
    SX128X_RADIO_SET_TXPARAMS             = 0x8E,
    SX128X_RADIO_SET_CADPARAMS            = 0x88,
    SX128X_RADIO_SET_BUFFERBASEADDRESS    = 0x8F,
    SX128X_RADIO_SET_MODULATIONPARAMS     = 0x8B,
    SX128X_RADIO_SET_PACKETPARAMS         = 0x8C,
    SX128X_RADIO_GET_RXBUFFERSTATUS       = 0x17,
    SX128X_RADIO_GET_PACKETSTATUS         = 0x1D,
    SX128X_RADIO_GET_RSSIINST             = 0x1F,
    SX128X_RADIO_SET_DIOIRQPARAMS         = 0x8D,
    SX128X_RADIO_GET_IRQSTATUS            = 0x15,
    SX128X_RADIO_CLR_IRQSTATUS            = 0x97,
    SX128X_RADIO_CALIBRATE                = 0x89,
    SX128X_RADIO_SET_REGULATORMODE        = 0x96,
    SX128X_RADIO_SET_SAVECONTEXT          = 0xD5,
    SX128X_RADIO_SET_AUTOTX               = 0x98,
    SX128X_RADIO_SET_AUTOFS               = 0x9E,
    SX128X_RADIO_SET_LONGPREAMBLE         = 0x9B,
    SX128X_RADIO_SET_UARTSPEED            = 0x9D,
    SX128X_RADIO_SET_RANGING_ROLE         = 0xA3,
}RadioCommands_t;

/**
 * @brief Structure describing the error codes for callback functions
 */
typedef enum
{
    SX128X_IRQ_HEADER_ERROR_CODE           = 0x00,
    SX128X_IRQ_SYNCWORD_ERROR_CODE,
    SX128X_IRQ_CRC_ERROR_CODE,
    SX128X_IRQ_RANGING_ON_LORA_ERROR_CODE,
}IrqErrorCode_t;

/**
 * @brief Structure describing the ranging codes for callback functions
 */
typedef enum
{
    SX128X_IRQ_RANGING_SLAVE_ERROR_CODE    = 0x00,
    SX128X_IRQ_RANGING_SLAVE_VALID_CODE,
    SX128X_IRQ_RANGING_MASTER_ERROR_CODE,
    SX128X_IRQ_RANGING_MASTER_VALID_CODE,
}IrqRangingCode_t;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _SX128X_REGISTERS_H_ */
/** @} */