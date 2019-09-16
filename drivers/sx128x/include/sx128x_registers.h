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
#define SX128X_REG_LR_FIRMWARE_VERSION_MSB              (0x0153)

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
 * @brief Represents all possible opcode understood by the radio
 */
typedef enum {
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
} sx128x_radio_commands_t;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* _SX128X_REGISTERS_H_ */
/** @} */