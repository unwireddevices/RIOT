/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_lis2hh12
 * @{
 *
 * @file
 * @brief       Command definition for the LIS2HH12 accelerometer
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#ifndef LIS2HH12_INTERNAL_H
#define LIS2HH12_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LIS2HH12 registers
 * @{
 */
#define LIS2HH12_TEMP_L                 (0x0B)
#define LIS2HH12_TEMP_H                 (0x0C)
#define LIS2HH12_WHO_AM_I               (0x0F)
#define LIS2HH12_ACT_THS                (0x1E)
#define LIS2HH12_ACT_DUR                (0x1F)
#define LIS2HH12_CTRL1                  (0x20)
#define LIS2HH12_CTRL2                  (0x21)
#define LIS2HH12_CTRL3                  (0x22)
#define LIS2HH12_CTRL4                  (0x23)
#define LIS2HH12_CTRL5                  (0x24)
#define LIS2HH12_CTRL6                  (0x25)
#define LIS2HH12_CTRL7                  (0x26)
#define LIS2HH12_STATUS                 (0x27)
#define LIS2HH12_OUT_X_L                (0x28)
#define LIS2HH12_OUT_X_H                (0x29)
#define LIS2HH12_OUT_Y_L                (0x2A)
#define LIS2HH12_OUT_Y_H                (0x2B)
#define LIS2HH12_OUT_Z_L                (0x2C)
#define LIS2HH12_OUT_Z_H                (0x2D)
#define LIS2HH12_FIFO_CTRL              (0x2E)
#define LIS2HH12_FIFO_SRC               (0x2F)
#define LIS2HH12_INT1_CFG               (0x30)
#define LIS2HH12_INT1_SRC               (0x31)
#define LIS2HH12_INT1_THS_X             (0x32)
#define LIS2HH12_INT1_THS_Y             (0x33)
#define LIS2HH12_INT1_THS_Z             (0x34)     
#define LIS2HH12_INT1_DURATION          (0x35)
#define LIS2HH12_INT2_CFG               (0x36)
#define LIS2HH12_INT2_SRC               (0x37)
#define LIS2HH12_INT2_THS               (0x38)
#define LIS2HH12_INT2_DURATION          (0x39)
#define LIS2HH12_REF_X_L                (0x3A)
#define LIS2HH12_REF_X_H                (0x3B)
#define LIS2HH12_REF_Y_L                (0x3C)
#define LIS2HH12_REF_Y_H                (0x3D)
#define LIS2HH12_REF_Z_L                (0x3E)
#define LIS2HH12_REF_Z_H                (0x3F)     
/** @} */

/**
 * @name    Selected register values
 * @{
 */
#define WHO_AM_I_VAL                    (0x41)
/** @} */


/**
 * @name    Interrrupt pins
 * @{
 */
#define LIS2HH12_INT1_PIN               (0x00)
#define LIS2HH12_INT2_PIN               (0x01)
/** @} */

/**
 * @name    Interrrupt bit position
 * @{
 */
#define LIS2HH12_INT_DATA_READY_BIT     (0x07)
#define LIS2HH12_INT_SINGLE_TAP_BIT     (0x06)
#define LIS2HH12_INT_DOUBLE_TAP_BIT     (0x05)
#define LIS2HH12_INT_ACTIVITY_BIT       (0x04)
#define LIS2HH12_INT_INACTIVITY_BIT     (0x03)
#define LIS2HH12_INT_FREE_FALL_BIT      (0x02)
#define LIS2HH12_INT_WATERMARK_BIT      (0x01)
#define LIS2HH12_INT_OVERRUNY_BIT       (0x00)

#define LIS2HH12_DATA_READY             (0x07)
#define LIS2HH12_SINGLE_TAP             (0x06)
#define LIS2HH12_DOUBLE_TAP             (0x05)
#define LIS2HH12_ACTIVITY               (0x04)
#define LIS2HH12_INACTIVITY             (0x03)
#define LIS2HH12_FREE_FALL              (0x02)
#define LIS2HH12_WATERMARK              (0x01)
#define LIS2HH12_OVERRUNY               (0x00)
/** @} */

/**
 * @name    LIS2HH12 masks for register CTRL1
 * @{
 */
#define LIS2HH12_MASK_CTRL1_X_EN        (0x01)
#define LIS2HH12_MASK_CTRL1_Y_EN        (0x02)
#define LIS2HH12_MASK_CTRL1_Z_EN        (0x04)
#define LIS2HH12_MASK_CTRL1_XYZ_EN      (LIS2HH12_MASK_CTRL1_X_EN | \
                                         LIS2HH12_MASK_CTRL1_Y_EN | \
                                         LIS2HH12_MASK_CTRL1_Z_EN)
#define LIS2HH12_MASK_CTRL1_BDU_EN      (0x08)
#define LIS2HH12_MASK_CTRL1_ODR         (0x70)
#define LIS2HH12_MASK_CTRL1_HR_EN       (0x80)   
/**  @} */

/**
 * @name    LIS2HH12 masks for register CTRL3
 * @{
 */
#define LIS2HH12_MASK_FIFO_EN           (0x80)
#define LIS2HH12_MASK_STOP_FTH          (0x40)
#define LIS2HH12_MASK_INT1_INACT        (0x20)
#define LIS2HH12_MASK_INT1_IG2          (0x10)
#define LIS2HH12_MASK_INT1_IG1          (0x08) 
#define LIS2HH12_MASK_INT1_OVR          (0x04) 
#define LIS2HH12_MASK_INT1_FTH          (0x02)
#define LIS2HH12_MASK_INT1_DRDY         (0x01)
/**  @} */

/**
 * @name    LIS2HH12 masks for register CTRL4
 * @{
 */
#define LIS2HH12_MASK_IF_ADD_INC_EN     (0x04)
/**  @} */

/**
 * @name    LIS2HH12 masks for register STATUS
 * @{
 */
#define LIS2HH12_MASK_ZYXOR             (0x80)      /**< X-, Y- and Z-axis data overrun.       */
#define LIS2HH12_MASK_ZOR               (0x40)      /**< Z-axis data overrun.                  */
#define LIS2HH12_MASK_YOR               (0x20)      /**< Y-axis data overrun.                  */
#define LIS2HH12_MASK_XOR               (0x10)      /**< X-axis data overrun.                  */
#define LIS2HH12_MASK_ZYXDA             (0x08)      /**< X-, Y- and Z-axis new data available. */
#define LIS2HH12_MASK_ZDA               (0x04)      /**< Z-axis new data available.            */
#define LIS2HH12_MASK_XDA               (0x02)      /**< Y-axis new data available.            */
#define LIS2HH12_MASK_YDA               (0x01)      /**< Y-axis new data available.            */
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* LIS2HH12_INTERNAL_H */
/** @} */