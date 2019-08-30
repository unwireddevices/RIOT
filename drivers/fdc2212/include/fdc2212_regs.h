/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fdc2212
 * @brief       Register definitions for the TI FDC2212 digital capacitor sensor
 * @author      Alexander Ugorelov <info@unwds.com>
 * @file
 * @{
 */

#ifndef __FDC2212_REGS_H__
#define __FDC2212_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name FDC2212 register addresses
 * @{
 */
#define FDC2212_REG_DATA_MSB_CH0                        (0x00)
#define FDC2212_REG_DATA_LSB_CH0                        (0x01)
#define FDC2212_REG_DATA_MSB_CH1                        (0x02)
#define FDC2212_REG_DATA_LSB_CH1                        (0x03)
#define FDC2212_REG_RCOUNT_CH0                          (0x08)
#define FDC2212_REG_RCOUNT_CH1                          (0x09)
#define FDC2212_REG_SETTLECOUNT_CH0                     (0x10)
#define FDC2212_REG_SETTLECOUNT_CH1                     (0x11)
#define FDC2212_REG_CLOCK_DIVIDERS_CH0                  (0x14)
#define FDC2212_REG_CLOCK_DIVIDERS_CH1                  (0x15)
#define FDC2212_REG_STATUS                              (0x18)
#define FDC2212_REG_STATUS_CONFIG                       (0x19)
#define FDC2212_REG_CONFIG                              (0x1A)
#define FDC2212_REG_MUX_CONFIG                          (0x1B)
#define FDC2212_REG_RESET_DEV                           (0x1C)
#define FDC2212_REG_DRIVE_CURRENT_CH0                   (0x1E)
#define FDC2212_REG_DRIVE_CURRENT_CH1                   (0x1F)
#define FDC2212_REG_MANUFACTURER_ID                     (0x7E)
#define FDC2212_REG_DEVICE_ID                           (0x7F)
/** @} */

/**
 * @name DATA_CH0 register bits
 * @{
 */
/* Channel 0 Conversion Watchdog Timeout Error Flag. Cleared by reading the bit. */
#define FDC2212_REG_DATA_MSB_CH0_ERR_WD_BIT             (13)
#define FDC2212_REG_DATA_MSB_CH0_ERR_WD_MASK            (1 << FDC2212_REG_CH0_ERR_WD_BIT)
/* Channel 0 Amplitude Warning. Cleared by reading the bit */
#define FDC2212_REG_DATA_MSB_CH0_ERR_AW_BIT             (12)
#define FDC2212_REG_DATA_MSB_CH0_ERR_AW_MASK            (1 << FDC2212_REG_CH0_ERR_AW_BIT)
/* Channel 0 Conversion Result */
#define FDC2212_REG_DATA_MSB_CH0_MASK                   (0x0FFF)
/** @} */

/**
 * @name DATA_LSB_CH0 register bits
 * @{
 */
/* Channel 0 Conversion Result */
#define FDC2212_REG_DATA_LSB_CH0_MASK                   (0xFFFF)
/** @} */

/**
 * @name DATA_CH1 register bits
 * @{
 */
/* Channel 1 Conversion Watchdog Timeout Error Flag. Cleared by reading the bit. */
#define FDC2212_REG_DATA_MSB_CH1_ERR_WD_BIT             (13)
#define FDC2212_REG_DATA_MSB_CH1_ERR_WD_MASK            (1 << FDC2212_REG_CH1_ERR_WD_BIT)
/* Channel 1 Amplitude Warning. Cleared by reading the bit */
#define FDC2212_REG_DATA_MSB_CH1_ERR_AW_BIT             (12)
#define FDC2212_REG_DATA_MSB_CH1_ERR_AW_MASK            (1 << FDC2212_REG_CH1_ERR_AW_BIT)
/* Channel 1 Conversion Result */
#define FDC2212_REG_DATA_MSB_CH1_MASK                   (0x0FFF)
/** @} */

/**
 * @name DATA_LSB_CH1 register bits
 * @{
 */
/* Channel 1 Conversion Result */
#define FDC2212_REG_DATA_LSB_CH1_MASK                   (0xFFFF)
/** @} */

/**
 * @name RCOUNT_CH0 register bits
 * @{
 */
/* Channel 0 Reference Count 
   Conversion Interval Time 
   0x0000-0x00FF: Reserved
   0x0100-0xFFFF: Conversion Time (tC0) = (CH0_RCOUNTˣ16)/fREF0 */
#define FDC2212_REG_RCOUNT_CH0_MASK                     (0xFFFF)
/** @} */

/**
 * @name RCOUNT_CH1 register bits
 * @{
 */
/* Channel 1 Reference Count 
   Conversion Interval Time 
   0x0000-0x00FF: Reserved
   0x0100-0xFFFF: Conversion Time (tC0) = (CH0_RCOUNTˣ16)/fREF0 */
#define FDC2212_REG_RCOUNT_CH1_MASK                     (0xFFFF)
/** @} */

/**
 * @name SETTLECOUNT_CH0 register bits
 * @{
 */
/* Channel 0 Conversion Settling
   The FDC will use this settling time to allow the LC sensor to
   stabilize before initiation of a conversion on Channel 0.
   If the amplitude has not settled prior to the conversion start, an
   Amplitude warning will be generated if reporting of this type of
   warning is enabled.
   b0000 0000 0000 0000: Settle Time (tS0)= 32 ÷ fREF0
   b0000 0000 0000 0001: Settle Time (tS0)= 32 ÷ fREF0
   b0000 0000 0000 0010 - b1111 1111 1111 1111: Settle Time
   (tS0)= (CH0_SETTLECOUNTˣ16) ÷ fREF0 */
#define FDC2212_REG_SETTLECOUNT_CH0_MASK                (0xFFFF)
/** @} */

/**
 * @name SETTLECOUNT_CH1 register bits
 * @{
 */
/* Channel 1 Conversion Settling
   The FDC will use this settling time to allow the LC sensor to
   stabilize before initiation of a conversion on Channel 1.
   If the amplitude has not settled prior to the conversion start, an
   Amplitude warning will be generated if reporting of this type of
   warning is enabled.
   b0000 0000 0000 0000: Settle Time (tS0)= 32 ÷ fREF0
   b0000 0000 0000 0001: Settle Time (tS0)= 32 ÷ fREF0
   b0000 0000 0000 0010 - b1111 1111 1111 1111: Settle Time
   (tS0)= (CH0_SETTLECOUNTˣ16) ÷ fREF0 */
#define FDC2212_REG_SETTLECOUNT_CH1_MASK                (0xFFFF)
/** @} */

/**
 * @name CLOCK_DIVIDERS_CH0 register bits
 * @{
 */
/* Channel 0 Sensor frequency select
   for differential sensor configuration:
   b01: divide by 1. Choose for sensor frequencies between
        0.01MHz and 8.75MHz
   b10: divide by 2. Choose for sensor frequencies between 5MHz
        and 10MHz for single-ended sensor configuration:
   b10: divide by 2. Choose for sensor frequencies between
        0.01MHz and 10MHz */
#define FDC2212_CLOCK_DIVIDERS_CH0_FIN_SEL_SHIFT        (12)
#define FDC2212_CLOCK_DIVIDERS_CH0_FIN_SEL_MASK         (0x3000)

/* Channel 0 Reference Divider Sets the divider for Channel 0
   reference. Use this to scale the maximum conversion frequency.
   b00’0000’0000: Reserved. Do not use.
   CH0_FREF_DIVIDER≥b00’0000’0001: fREF0 = fCLK/CH0_FREF_DIVIDER */
#define FDC2212_CLOCK_DIVIDERS_CH0_FREF_DIVIDER_SHIFT   (0)
#define FDC2212_CLOCK_DIVIDERS_CH0_FREF_DIVIDER_MASK    (0x03FF)
/** @} */

/**
 * @name CLOCK_DIVIDERS_CH1 register bits
 * @{
 */
/* Channel 1 Sensor frequency select
   for differential sensor configuration:
   b01: divide by 1. Choose for sensor frequencies between
        0.01MHz and 8.75MHz
   b10: divide by 2. Choose for sensor frequencies between 5MHz
        and 10MHz for single-ended sensor configuration:
   b10: divide by 2. Choose for sensor frequencies between
        0.01MHz and 10MHz */
#define FDC2212_CLOCK_DIVIDERS_CH1_FIN_SEL_SHIFT        (12)
#define FDC2212_CLOCK_DIVIDERS_CH1_FIN_SEL_MASK         (0x3000)

/* Channel 1 Reference Divider Sets the divider for Channel 1
   reference. Use this to scale the maximum conversion frequency.
   b00’0000’0000: Reserved. Do not use.
   CH1_FREF_DIVIDER≥b00’0000’0001: fREF0 = fCLK/CH0_FREF_DIVIDER */
#define FDC2212_CLOCK_DIVIDERS_CH1_FREF_DIVIDER_SHIFT   (0)
#define FDC2212_CLOCK_DIVIDERS_CH1_FREF_DIVIDER_MASK    (0x03FF)
/** @} */

/**
 * @name STATUS register bits
 * @{
 */
/* Error Channel
   Indicates which channel has generated a Flag or Error. Once
   flagged, any reported error is latched and maintained until either
   the STATUS register or the DATA_CHx register corresponding
   to the Error Channel is read.
   b00: Channel 0 is source of flag or error.
   b01: Channel 1 is source of flag or error. */
#define FDC2212_REG_STATUS_ERR_CHAN_SHIFT               (14)
#define FDC2212_REG_STATUS_ERR_CHAN_MASK                (0xC000)
/* Watchdog Timeout Error
   b0: No Watchdog Timeout error was recorded since the last
       read of the STATUS register.
   b1: An active channel has generated a Watchdog Timeout error.
   Refer to STATUS.ERR_CHAN field to determine which channel
   is the source of this error. */
#define FDC2212_REG_STATUS_ERR_WD_BIT                   (11)
#define FDC2212_REG_STATUS_ERR_WD_MASK                  (1 << FDC2212_REG_STATUS_ERR_WD_BIT)
/* Amplitude High Warning
   b0: No Amplitude High warning was recorded since the last read
       of the STATUS register.
   b1: An active channel has generated an Amplitude High
       warning. 
    Refer to STATUS.ERR_CHAN field to determine which
    channel is the source of this warning. */
#define FDC2212_REG_STATUS_ERR_AHW_BIT                  (10)
#define FDC2212_REG_STATUS_ERR_AHW_MASK                 (1 << FDC2212_REG_STATUS_ERR_AHW_BIT)
/* Amplitude Low Warning
   b0: No Amplitude Low warning was recorded since the last read
       of the STATUS register.
   b1: An active channel has generated an Amplitude Low warning.
   Refer to STATUS.ERR_CHAN field to determine which channel
   is the source of this warning. */
#define FDC2212_REG_STATUS_ERR_ALW_BIT                  (9)
#define FDC2212_REG_STATUS_ERR_ALW_MASK                 (1 << FDC2212_REG_STATUS_ERR_ALW_BIT)

#define FDC2212_REG_STATUS_ERROR                        (FDC2212_REG_STATUS_ERR_WD_MASK | \
                                                        FDC2212_REG_STATUS_ERR_AHW_MASK | \
                                                        FDC2212_REG_STATUS_ERR_ALW_MASK)

/* Data Ready Flag.
   b0: No new conversion result was recorded in the STATUS
       register.
   b1: A new conversion result is ready. 
   When in Single Channel Conversion, this indicates a single conversion is available. 
   When in sequential mode, this indicates that a new conversion result
   for all active channels is now available. */
#define FDC2212_REG_STATUS_DATA_RDY_BIT                 (6)
#define FDC2212_REG_STATUS_DATA_RDY_MASK                (1 << FDC2212_REG_STATUS_DATA_RDY_BIT)
/* Channel 0 Unread Conversion 
   b0: No unread conversion is present for Channel 0.
   b1: An unread conversion is present for Channel 0.
   Read Register DATA_CH0 to retrieve conversion results. */
#define FDC2212_REG_STATUS_CH0_UNREADCONV_BIT           (3)
#define FDC2212_REG_STATUS_CH0_UNREADCONV_MASK          (1 << FDC2212_REG_STATUS_CH0_UNREADCONV_BIT)
/* Channel 1 Unread Conversion 
   b0: No unread conversion is present for Channel 1.
   b1: An unread conversion is present for Channel 1.
   Read Register DATA_CH1 to retrieve conversion results. */
#define FDC2212_REG_STATUS_CH1_UNREADCONV_BIT           (2)
#define FDC2212_REG_STATUS_CH1_UNREADCONV_MASK          (1 << FDC2212_REG_STATUS_CH1_UNREADCONV_BIT)
/** @} */

/**
 * @name STATUS_CONFIG (ERROR_CONFIG) register bits
 * @{
 */
/* Watchdog Timeout Error to Output Register
   b0: Do not report Watchdog Timeout errors in the DATA_CHx
       registers.
   b1: Report Watchdog Timeout errors in the 
       DATA_CHx.CHx_ERR_WD register field corresponding to the
       channel that generated the error. */
#define FDC2212_REG_STATUS_CONFIG_WD_ERR2OUT_BIT        (13)
#define FDC2212_REG_STATUS_CONFIG_WD_ERR2OUT_MASK       (1 << FDC2212_REG_STATUS_CONFIG_WD_ERR2OUT_BIT)
/* Amplitude High Warning to Output Register
   b0: Do not report Amplitude High warnings in the DATA_CHx
       registers.
   b1: Report Amplitude High warnings in the
       DATA_CHx.CHx_ERR_AW register field corresponding to the
       channel that generated the warning. */
#define FDC2212_REG_STATUS_CONFIG_AH_WARN2OUT_BIT       (12)
#define FDC2212_REG_STATUS_CONFIG_AH_WARN2OUT_MASK      (1 << FDC2212_REG_STATUS_CONFIG_AH_WARN2OUT_BIT)
/* Amplitude Low Warning to Output Register
   b0: Do not report Amplitude Low warnings in the DATA_CHx
       registers.
   b1: Report Amplitude High warnings in the
       DATA_CHx.CHx_ERR_AW register field corresponding to the
       channel that generated the warning. */
#define FDC2212_REG_STATUS_CONFIG_AL_WARN2OUT_BIT       (11)
#define FDC2212_REG_STATUS_CONFIG_AL_WARN2OUT_MASK      (1 << FDC2212_REG_STATUS_CONFIG_AL_WARN2OUT_BIT)
/* Watchdog Timeout Error to INTB 
   b0: Do not report Under-range
       errors by asserting INTB pin and STATUS register.
   b1: Report Watchdog Timeout errors by asserting INTB pin and
       updating STATUS.ERR_WD register field. */
#define FDC2212_REG_STATUS_CONFIG_WD_ERR2INT_BIT        (5)
#define FDC2212_REG_STATUS_CONFIG_WD_ERR2INT_MASK       (1 << FDC2212_REG_STATUS_CONFIG_WD_ERR2INT_BIT)
/* Data Ready Flag to INTB 
   b0: Do not report Data Ready Flag by
       asserting INTB pin and STATUS register.
   b1: Report Data Ready Flag by asserting INTB pin and updating
       STATUS.DRDY register field.*/
#define FDC2212_REG_STATUS_CONFIG_DATA_RDY_2INT_BIT     (0)
#define FDC2212_REG_STATUS_CONFIG_DATA_RDY_2INT_MASK    (1 << FDC2212_REG_STATUS_CONFIG_DATA_RDY_2INT_BIT)
/** @} */

/**
 * @name CONFIG register bits
 * @{
 */
/* Active Channel Selection
   Selects channel for continuous conversions when
   MUX_CONFIG.SEQUENTIAL is 0.
   b00: Perform continuous conversions on Channel 0
   b01: Perform continuous conversions on Channel 1
*/
#define FDC2212_REG_CONFIG_ACTIVE_CHAN_SHIFT            (14)
#define FDC2212_REG_CONFIG_ACTIVE_CHAN_MASK             (0xC000)
/* Sleep Mode Enable
   Enter or exit low power Sleep Mode.
   b0: Device is active.
   b1: Device is in Sleep Mode. (default) */
#define FDC2212_REG_CONFIG_SLEEP_MODE_EN_BIT            (13)
#define FDC2212_REG_CONFIG_SLEEP_MODE_EN_MASK           (1 << FDC2212_REG_CONFIG_SLEEP_MODE_EN_BIT)
/* Reserved Set to b1 */
#define FDC2212_REG_CONFIG_OP_MODE0_BIT                 (12)
#define FDC2212_REG_CONFIG_OP_MODE0_MASK                (1 << FDC2212_REG_CONFIG_OP_MODE0_BIT)
/* Sensor Activation Mode Selection.
   Set the mode for sensor initialization.
   b0: Full Current Activation Mode – the FDC will drive maximum
       sensor current for a shorter sensor activation time.
   b1: Low Power Activation Mode – the FDC uses the value
       programmed in DRIVE_CURRENT_CHx during sensor
       activation to minimize power consumption.*/
#define FDC2212_REG_CONFIG_SENSOR_ACTIVATE_SEL_BIT      (11)
#define FDC2212_REG_CONFIG_SENSOR_ACTIVATE_SEL_MASK     (1 << FDC2212_REG_CONFIG_SENSOR_ACTIVATE_SEL_BIT)
/* Reserved Set to b1 */
#define FDC2212_REG_CONFIG_OP_MODE1_BIT                 (10)
#define FDC2212_REG_CONFIG_OP_MODE1_MASK                (1 << FDC2212_REG_CONFIG_OP_MODE1_BIT)
/* Select Reference Frequency Source
   b0: Use Internal oscillator as reference frequency
   b1: Reference frequency is provided from CLKIN pin.*/
#define FDC2212_REG_CONFIG_REF_CLK_SRC_BIT              (9)
#define FDC2212_REG_CONFIG_REF_CLK_SRC_MASK             (1 << FDC2212_REG_CONFIG_REF_CLK_SRC_BIT)
/* Reserved Set to b0 */
#define FDC2212_REG_CONFIG_OP_MODE2_BIT                 (8)
#define FDC2212_REG_CONFIG_OP_MODE2_MASK                (1 << FDC2212_REG_CONFIG_OP_MODE2_BIT)
/* INTB Disable
   b0: INTB pin will be asserted when status register updates.
   b1: INTB pin will not be asserted when status register updates. */
#define FDC2212_REG_CONFIG_INTB_DIS_BIT                 (7)
#define FDC2212_REG_CONFIG_INTB_DIS_MASK                (1 << FDC2212_REG_CONFIG_INTB_DIS_BIT)
/* High Current Sensor Drive
   b0: The FDC will drive all channels with normal sensor current
       (1.5mA max).
   b1: The FDC will drive channel 0 with current >1.5mA.
   This mode is not supported if AUTOSCAN_EN = b1 (multichannel mode). */
#define FDC2212_REG_CONFIG_HIGH_CURRENT_DRV_BIT         (6)
#define FDC2212_REG_CONFIG_HIGH_CURRENT_DRV_MASK        (1 << FDC2212_REG_CONFIG_HIGH_CURRENT_DRV_BIT)
/* Reserved Set to b00’0001*/
#define FDC2212_REG_CONFIG_OP_MODE3_SHIFT               (0)
#define FDC2212_REG_CONFIG_OP_MODE3_MASK                (0x0001)
/** @} */

/**
 * @name MUX_CONFIG register bits
 * @{
 */
/* Auto-Scan Mode Enable
   b0: Continuous conversion on the single channel selected by
       CONFIG.ACTIVE_CHAN register field.
   b1: Auto-Scan conversions as selected by
       MUX_CONFIG.RR_SEQUENCE register field.*/
#define FDC2212_REG_MUX_CONFIG_AUTOSCAN_EN_BIT          (15)
#define FDC2212_REG_MUX_CONFIG_AUTOSCAN_EN_MASK         (1 << FDC2212_REG_MUX_CONFIG_AUTOSCAN_EN_BIT)
/* Auto-Scan Sequence Configuration Configure multiplexing
   channel sequence. The FDC will perform a single conversion on
   each channel in the sequence selected, and then restart the
   sequence continuously.
   b00: Ch0, Ch1
   b11: Ch0, Ch1
*/
#define FDC2212_REG_MUX_CONFIG_RR_SEQUENCE_SHIFT        (13)
#define FDC2212_REG_MUX_CONFIG_RR_SEQUENCE_MASK         (0x6000)
/* Reserved. Must be set to 00 0100 0001 (0x41) */
#define FDC2212_REG_MUX_CONFIG_RESERVED_SHIFT           (3)
#define FDC2212_REG_MUX_CONFIG_RESERVED_MASK            (0x41 << FDC2212_REG_MUX_CONFIG_RESERVED_SHIFT)
/* Input deglitch filter bandwidth.
   Select the lowest setting that exceeds the oscillation tank
   oscillation frequency.
   b001: 1MHz
   b100: 3.3MHz
   b101: 10MHz
   b111: 33MHz
*/
#define FDC2212_REG_MUX_CONFIG_DEGLITCH_SHIFT           (0)
#define FDC2212_REG_MUX_CONFIG_DEGLITCH_MASK            (0x0007)
/** @} */

/**
 * @name RESET_DEV register bits
 * @{
 */
/* Device Reset
   Write b1 to reset the device. Will always readback 0. */
#define FDC2212_REG_RESET_DEV_BIT                       (15)
#define FDC2212_REG_RESET_DEV_MASK                      (1 << FDC2212_REG_RESET_DEV_BIT)
/* Output gain control (FDC2112, FDC2114 only)
   00: Gain =1 (0 bits shift)
   01: Gain = 4 (2 bits shift)
   10: Gain = 8 (3 bits shift)
   11: Gain = 16 (4 bits shift)*/
#define FDC2212_REG_RESET_DEV_OUTPUT_GAIN_SHIFT         (9)
#define FDC2212_REG_RESET_DEV_OUTPUT_GAIN_MASK          (0x0600)
/** @} */

/**
 * @name DRIVE_CURRENT_CH0 register bits
 * @{
 */
/* Channel 0 Sensor drive current
   This field defines the Drive Current used during the settling +
   conversion time of Channel 0 sensor clock. Set such that 1.2V ≤
   sensor oscillation amplitude (pk) ≤ 1.8V
   00000: 0.016mA
   00001: 0.018mA
   00010: 0.021mA
   00011: 0.025mA
   00100: 0.028mA
   00101: 0.033mA
   00110: 0.038mA
   00111: 0.044mA
   01000: 0.052mA
   01001: 0.060mA
   01010: 0.069mA
   01011: 0.081mA
   01100: 0.093mA
   01101: 0.108mA
   01110: 0.126mA
   01111: 0.146mA
   10000: 0.169mA
   10001: 0.196mA
   10010: 0.228mA
   10011: 0.264mA
   10100: 0.307mA
   10101: 0.356mA
   10110: 0.413mA
   10111: 0.479mA
   11000: 0.555mA
   11001: 0.644mA
   11010: 0.747mA
   11011: 0.867mA
   11100: 1.006mA
   11101: 1.167mA
   11110: 1.354mA
   11111: 1.571mA
*/
#define FDC2212_REG_DRIVE_CURRENT_CH0_IDRIVE_SHIFT      (11)
#define FDC2212_REG_DRIVE_CURRENT_CH0_IDRIVE_MASK       (0xF800)
/** @} */

/**
 * @name DRIVE_CURRENT_CH0 register bits
 * @{
 */
/* Channel 0 Sensor drive current
   This field defines the Drive Current used during the settling +
   conversion time of Channel 0 sensor clock. Set such that 1.2V ≤
   sensor oscillation amplitude (pk) ≤ 1.8V
   00000: 0.016mA
   00001: 0.018mA
   00010: 0.021mA
   ...
   11110: 1.354mA
   11111: 1.571mA
*/
#define FDC2212_REG_DRIVE_CURRENT_CH1_IDRIVE_SHIFT      (11)
#define FDC2212_REG_DRIVE_CURRENT_CH1_IDRIVE_MASK       (0xF800)
/** @} */

/**
 * @name MANUFACTURER_ID register bits
 * @{
 */
/* Manufacturer ID = 0x5449 */
#define FDC2212_REG_MANUFACTURER_ID_MASK                (0xFFFF)
#define FDC2212_MANUFACTURER_ID                         (0x5449)
/** @} */

/**
 * @name DEVICE_ID register bits
 * @{
 */
/* Device ID 0x3055 (FDC2212, FDC2214 only) */
#define FDC2212_REG_DEVICE_ID_MASK                      (0xFFFF)
#define FDC2212_DEVICE_ID                               (0x3055)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __FDC2212_REGS_H__ */
/** @} */