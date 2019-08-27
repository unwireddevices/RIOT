/*
 * Copyright (C) 2019 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis2dh12
 * @{
 *
 * @file
 * @brief       Registers definition for the STMicro LIS2DH12 accelerometer
 *
 * @author      Alexander Ugorelov <info@unwds.com>
 */

#ifndef LIS2DH12_INTERNAL_H
#define LIS2DH12_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    LIS2DH12 registers
 * @{
 */

/**
 * @brief   Aux status register
 */
#define LIS2DH12_STATUS_REG_AUX                         (0x07)
#define LIS2DH12_STATUS_REG_AUX_TDO_BIT                 (6)
#define LIS2DH12_STATUS_REG_AUX_TDO_MASK                (1 << LIS2DH12_STATUS_AUX_TDO_BIT)
#define LIS2DH12_STATUS_REG_AUX_TDA_BIT                 (2)
#define LIS2DH12_STATUS_REG_AUX_TDA_MASK                (1 << LIS2DH12_STATUS_AUX_TDA_BIT)

/**
 * @brief   Temperature sensor data register
 */
#define LIS2DH12_OUT_TEMP_L                             (0x0C)
#define LIS2DH12_OUT_TEMP_H                             (0x0D)

/**
 * @brief   Identifier register
 */
#define LIS2DH12_WHO_AM_I                               (0x0F)

/**
 * @brief   Identifier register value
 *
 * The WHO_AM_I register should contain this value in order to correctly
 * identify the chip.
 */
#define LIS2DH12_WHO_AM_I_RESPONSE                      (0x33)

/**
 * @brief   Control register 0
 */
#define LIS2DH12_CTRL_REG0                              (0x1E)
#define LIS2DH12_CTRL_REG0_SDO_PU_DISC_BIT              (7)
#define LIS2DH12_CTRL_REG0_SDO_PU_DISC_MASK             (1 << LIS2DH12_CTRL_REG0_SDO_PU_DISC_BIT)
#define LIS2DH12_CTRL_REG0_CORRECT_OPER_BIT             (4)
#define LIS2DH12_CTRL_REG0_CORRECT_OPER_MASK            (1 << LIS2DH12_CTRL_REG0_CORRECT_OPER_BIT)

/**
 * @brief   Temperature configuration register
 */
#define LIS2DH12_TEMP_CFG_REG                           (0x1F)
#define LIS2DH12_TEMP_CFG_REG_TEMP_EN1_BIT              (7)
#define LIS2DH12_TEMP_CFG_REG_TEMP_EN1_MASK             (1 << LIS2DH12_TEMP_CFG_REG_TEMP_EN1_BIT)
#define LIS2DH12_TEMP_CFG_REG_TEMP_EN0_BIT              (6)
#define LIS2DH12_TEMP_CFG_REG_TEMP_EN0_MASK             (1 << LIS2DH12_TEMP_CFG_REG_TEMP_EN0_BIT)
#define LIS2DH12_TEMP_CFG_REG_TEMP_EN_MASK              (0xC0)

/**
 * @brief   Control register 1
 */
#define LIS2DH12_CTRL_REG1                              (0x20)
#define LIS2DH12_CTRL_REG1_ODR_SHIFT                    (4)
#define LIS2DH12_CTRL_REG1_ODR_MASK                     (0xF0)
#define LIS2DH12_CTRL_REG1_LPEN_BIT                     (3)
#define LIS2DH12_CTRL_REG1_LPEN_MASK                    (1 << LIS2DH12_CTRL_REG1_LPEN_BIT)
#define LIS2DH12_CTRL_REG1_ZEN_BIT                      (2)
#define LIS2DH12_CTRL_REG1_ZEN_MASK                     (1 << LIS2DH12_CTRL_REG1_ZEN_BIT)
#define LIS2DH12_CTRL_REG1_YEN_BIT                      (1)
#define LIS2DH12_CTRL_REG1_YEN_MASK                     (1 << LIS2DH12_CTRL_REG1_YEN_BIT)
#define LIS2DH12_CTRL_REG1_XEN_BIT                      (0)
#define LIS2DH12_CTRL_REG1_XEN_MASK                     (1 << LIS2DH12_CTRL_REG1_XEN_BIT)
#define LIS2DH12_CTRL_REG1_XYZEN_MASK                   (0x07)

/**
 * @brief   Control register 2
 */
#define LIS2DH12_CTRL_REG2                              (0x21)
#define LIS2DH12_CTRL_REG2_HPM_SHIFT                    (6)
#define LIS2DH12_CTRL_REG2_HPM_MASK                     (0xC0)
#define LIS2DH12_CTRL_REG2_HPCF_SHIFT                   (4)
#define LIS2DH12_CTRL_REG2_HCPF_MASK                    (0x30)
#define LIS2DH12_CTRL_REG2_FDS_BIT                      (3)
#define LIS2DH12_CTRL_REG2_FDS_MASK                     (1 << LIS2DH12_CTRL_REG2_FDS_BIT)
#define LIS2DH12_CTRL_REG2_HPCLICK_BIT                  (2)
#define LIS2DH12_CTRL_REG2_HPCLICK_MASK                 (1 << LIS2DH12_CTRL_REG2_HPCLICK_BIT)
#define LIS2DH12_CTRL_REG2_HP_IA2_BIT                   (1)
#define LIS2DH12_CTRL_REG2_HP_IA2_MASK                  (1 << LIS2DH12_CTRL_REG2_HP_IA2_BIT)
#define LIS2DH12_CTRL_REG2_HP_IA1_BIT                   (0)
#define LIS2DH12_CTRL_REG2_HP_IA1_MASK                  (1 << LIS2DH12_CTRL_REG2_HP_IA1_BIT)

/**
 * @brief   Control register 3
 */
#define LIS2DH12_CTRL_REG3                              (0x22)
#define LIS2DH12_CTRL_REG3_I1_CLICK_BIT                 (7)
#define LIS2DH12_CTRL_REG3_I1_CLICK_MASK                (1 << LIS2DH12_CTRL_REG3_I1_CLICK_BIT)
#define LIS2DH12_CTRL_REG3_I1_IA1_BIT                   (6)
#define LIS2DH12_CTRL_REG3_I1_IA1_MASK                  (1 << LIS2DH12_CTRL_REG3_I1_IA1_BIT)
#define LIS2DH12_CTRL_REG3_I1_IA2_BIT                   (5)
#define LIS2DH12_CTRL_REG3_I1_IA2_MASK                  (1 << LIS2DH12_CTRL_REG3_I1_IA2_BIT)
#define LIS2DH12_CTRL_REG3_I1_ZYXDA_BIT                 (4)
#define LIS2DH12_CTRL_REG3_I1_ZYXDA_MASK                (1 << LIS2DH12_CTRL_REG3_I1_ZYXDA_BIT)
#define LIS2DH12_CTRL_REG3_I1_WTM_BIT                   (2)
#define LIS2DH12_CTRL_REG3_I1_WTM_MASK                  (1 << LIS2DH12_CTRL_REG3_I1_WTM_BIT)
#define LIS2DH12_CTRL_REG3_I1_OVERRUN_BIT               (1)
#define LIS2DH12_CTRL_REG3_I1_OVERRUN_MASK              (1 << LIS2DH12_CTRL_REG3_I1_OVERRUN_BIT)

/**
 * @brief   Control register 4
 */
#define LIS2DH12_CTRL_REG4                              (0x23)
#define LIS2DH12_CTRL_REG4_BDU_BIT                      (7)
#define LIS2DH12_CTRL_REG4_BDU_MASK                     (1 << LIS2DH12_CTRL_REG4_BDU_BIT)
#define LIS2DH12_CTRL_REG4_BLE_BIT                      (6)
#define LIS2DH12_CTRL_REG4_BLE_MASK                     (1 << LIS2DH12_CTRL_REG4_BLE_BIT)
#define LIS2DH12_CTRL_REG4_FS_SHIFT                     (4)
#define LIS2DH12_CTRL_REG4_FS_MASK                      (0x30)
#define LIS2DH12_CTRL_REG4_HR_BIT                       (3)
#define LIS2DH12_CTRL_REG4_HR_MASK                      (1 << LIS2DH12_CTRL_REG4_HR_BIT)
#define LIS2DH12_CTRL_REG4_ST_SHIFT                     (1)
#define LIS2DH12_CTRL_REG4_ST_MASK                      (1 << LIS2DH12_CTRL_REG4_ST_SHIFT)
#define LIS2DH12_CTRL_REG4_SIM_BIT                      (0)
#define LIS2DH12_CTRL_REG4_SIM_MASK                     (1 << LIS2DH12_CTRL_REG4_SIM_BIT)

/**
 * @brief   Control register 5
 */
#define LIS2DH12_CTRL_REG5                              (0x24)
#define LIS2DH12_CTRL_REG5_BOOT_BIT                     (7)
#define LIS2DH12_CTRL_REG5_BOOT_MASK                    (1 << LIS2DH12_CTRL_REG5_BOOT_BIT)
#define LIS2DH12_CTRL_REG5_FIFO_EN_BIT                  (6)
#define LIS2DH12_CTRL_REG5_FIFO_EN_MASK                 (1 << LIS2DH12_CTRL_REG5_FIFO_EN_MASK)
#define LIS2DH12_CTRL_REG5_LIR_INT1_BIT                 (3)
#define LIS2DH12_CTRL_REG5_LIR_INT1_MASK                (1 << LIS2DH12_CTRL_REG5_LIR_INT1_BIT)
#define LIS2DH12_CTRL_REG5_D4D_INT1_BIT                 (2)
#define LIS2DH12_CTRL_REG5_D4D_INT1_MASK                (1 << LIS2DH12_CTRL_REG5_D4D_INT1_BIT)
#define LIS2DH12_CTRL_REG5_LIR_INT2_BIT                 (1)
#define LIS2DH12_CTRL_REG5_LIR_INT2_MASK                (1 << LIS2DH12_CTRL_REG5_LIR_INT2_BIT)
#define LIS2DH12_CTRL_REG5_D4D_INT2_BIT                 (0)
#define LIS2DH12_CTRL_REG5_D4D_INT2_MASK                (1 << LIS2DH12_CTRL_REG5_D4D_INT2_BIT)

/**
 * @brief   Control register 6
 */
#define LIS2DH12_CTRL_REG6                              (0x25)
#define LIS2DH12_CTRL_REG6_I2_CLICK_BIT                 (7)
#define LIS2DH12_CTRL_REG6_I2_CLICK_MASK                (1 << LIS2DH12_CTRL_REG6_I2_CLICK_BIT)
#define LIS2DH12_CTRL_REG6_I2_IA1_BIT                   (6)
#define LIS2DH12_CTRL_REG6_I2_IA1_MASK                  (1 << LIS2DH12_CTRL_REG6_I2_IA1_BIT)
#define LIS2DH12_CTRL_REG6_I2_IA2_BIT                   (5)
#define LIS2DH12_CTRL_REG6_I2_IA2_MASK                  (1 << LIS2DH12_CTRL_REG6_I2_IA2_BIT)
#define LIS2DH12_CTRL_REG6_I2_BOOT_BIT                  (4)
#define LIS2DH12_CTRL_REG6_I2_BOOT_MASK                 (1 << LIS2DH12_CTRL_REG6_I2_BOOT_BIT)
#define LIS2DH12_CTRL_REG6_I2_ACT_BIT                   (3)
#define LIS2DH12_CTRL_REG6_I2_ACT_MASK                  (1 << LIS2DH12_CTRL_REG6_I2_ACT_BIT)
#define LIS2DH12_CTRL_REG6_INT_POLARITY_BIT             (1)
#define LIS2DH12_CTRL_REG6_INT_POLARITY_MASK            (1 << LIS2DH12_CTRL_REG6_INT_POLARITY_BIT)

/**
 * @brief   Reference/datacapture register
 */
#define LIS2DH12_REG_REFERENCE                          (0x26)
#define LIS2DH12_REG_REFERENCE_REF_SHIFT                (0)
#define LIS2DH12_REG_REFERENCE_REF_MASK                 (0xFF)

/**
 * @brief   Axis status register
 */
#define LIS2DH12_STATUS_REG                             (0x27)
#define LIS2DH12_STATUS_REG_ZYXOR_BIT                   (7)
#define LIS2DH12_STATUS_REG_ZYXOR_MASK                  (1 << LIS2DH12_STATUS_REG_ZYXOR_BIT)
#define LIS2DH12_STATUS_REG_ZOR_BIT                     (6)
#define LIS2DH12_STATUS_REG_ZOR_MASK                    (1 << LIS2DH12_STATUS_REG_ZOR_BIT)
#define LIS2DH12_STATUS_REG_YOR_BIT                     (5)
#define LIS2DH12_STATUS_REG_YOR_MASK                    (1 << LIS2DH12_STATUS_REG_YOR_BIT)
#define LIS2DH12_STATUS_REG_XOR_BIT                     (4)
#define LIS2DH12_STATUS_REG_XOR_MASK                    (1 << LIS2DH12_STATUS_REG_XOR_BIT)
#define LIS2DH12_STATUS_REG_ZYXDA_BIT                   (3)
#define LIS2DH12_STATUS_REG_ZYXDA_MASK                  (1 << LIS2DH12_STATUS_REG_ZYXDA_BIT)
#define LIS2DH12_STATUS_REG_ZDA_BIT                     (2)
#define LIS2DH12_STATUS_REG_ZDA_MASK                    (1 << LIS2DH12_STATUS_REG_ZDA_BIT)
#define LIS2DH12_STATUS_REG_YDA_BIT                     (1)
#define LIS2DH12_STATUS_REG_YDA_MASK                    (1 << LIS2DH12_STATUS_REG_YDA_BIT)
#define LIS2DH12_STATUS_REG_XDA_BIT                     (0)
#define LIS2DH12_STATUS_REG_XDA_MASK                    (1 << LIS2DH12_STATUS_REG_XDA_BIT)

/**
 * @brief Acceleration data registers
 */
#define LIS2DH12_REG_OUT_X_L                            (0x28)
#define LIS2DH12_REG_OUT_X_H                            (0x29)
#define LIS2DH12_REG_OUT_Y_L                            (0x2A)
#define LIS2DH12_REG_OUT_Y_H                            (0x2B)
#define LIS2DH12_REG_OUT_Z_L                            (0x2C)
#define LIS2DH12_REG_OUT_Z_H                            (0x2D)

/**
 * @brief   FIFO control register
 */
#define LIS2DH12_FIFO_CTRL_REG                          (0x2E)
#define LIS2DH12_FIFO_CTRL_REG_FM_SHIFT                 (6)
#define LIS2DH12_FIFO_CTRL_REG_FM_MASK                  (0xC0)
#define LIS2DH12_FIFO_CTRL_REG_TR_BIT                   (5)
#define LIS2DH12_FIFO_CTRL_REG_TR_MASK                  (1 << LIS2DH12_FIFO_CTRL_REG_TR_BIT)
#define LIS2DH12_FIFO_CTRL_REG_FTH_SHIFT                (0)
#define LIS2DH12_FIFO_CTRL_REG_FTH_MASK                 (0x1F)

/**
 * @brief FIFO source register
 */
#define LIS2DH12_FIFO_SRC_REG                           (0x2F)
#define LIS2DH12_FIFO_SRC_REG_WTM_BIT                   (7)
#define LIS2DH12_FIFO_SRC_REG_WTM_MASK                  (1 << LIS2DH12_FIFO_SRC_REG_WTM_BIT)
#define LIS2DH12_FIFO_SRC_REG_OVRN_FIFO_BIT             (6)
#define LIS2DH12_FIFO_SRC_REG_OVRN_FIFO_MASK            (1 << LIS2DH12_FIFO_SRC_REG_OVRN_FIFO_BIT)
#define LIS2DH12_FIFO_SRC_REG_EMPTY_BIT                 (5)
#define LIS2DH12_FIFO_SRC_REG_EMPTY_MASK                (1 << LIS2DH12_FIFO_SRC_REG_EMPTY_BIT)
#define LIS2DH12_FIFO_SRC_REG_FSS_SHIFT                 (0)
#define LIS2DH12_FIFO_SRC_REG_FSS_MASK                  (0x1F)

/**
 * @brief   Interrupt 1 configuration register
 */
#define LIS2DH12_REG_INT1_CFG                           (0x30)
#define LIS2DH12_REG_INT1_CFG_AND_OR_BIT                (7)
#define LIS2DH12_REG_INT1_CFG_AND_OR_MASK               (1 << LIS2DH12_REG_INT1_CFG_AND_OR_BIT)
#define LIS2DH12_REG_INT1_CFG_INT_6D_BIT                (6)
#define LIS2DH12_REG_INT1_CFG_INT_6D_MASK               (1 << LIS2DH12_REG_INT1_CFG_INT_6D_BIT)
#define LIS2DH12_REG_INT1_CFG_ZHIE_BIT                  (5)
#define LIS2DH12_REG_INT1_CFG_ZHIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_ZHIE_BIT)
#define LIS2DH12_REG_INT1_CFG_ZLIE_BIT                  (4)
#define LIS2DH12_REG_INT1_CFG_ZLIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_ZLIE_BIT)
#define LIS2DH12_REG_INT1_CFG_YHIE_BIT                  (3)
#define LIS2DH12_REG_INT1_CFG_YHIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_YHIE_BIT)
#define LIS2DH12_REG_INT1_CFG_YLIE_BIT                  (2)
#define LIS2DH12_REG_INT1_CFG_YLIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_YLIE_BIT)
#define LIS2DH12_REG_INT1_CFG_XHIE_BIT                  (1)
#define LIS2DH12_REG_INT1_CFG_XHIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_XHIE_BIT)
#define LIS2DH12_REG_INT1_CFG_XLIE_BIT                  (0)
#define LIS2DH12_REG_INT1_CFG_XLIE_MASK                 (1 << LIS2DH12_REG_INT1_CFG_XLIE_BIT)

/**
 * @brief   Interrupt 1 source register
 */
#define LIS2DH12_REG_INT1_SRC                           (0x31)
#define LIS2DH12_REG_INT1_SRC_IA_BIT                    (6)
#define LIS2DH12_REG_INT1_SRC_IA_MASK                   (1 << LIS2DH12_REG_INT1_SRC_IA_BIT)
#define LIS2DH12_REG_INT1_SRC_ZH_BIT                    (5)
#define LIS2DH12_REG_INT1_SRC_ZH_MASK                   (1 << LIS2DH12_REG_INT1_SRC_ZH_BIT)
#define LIS2DH12_REG_INT1_SRC_ZL_BIT                    (4)
#define LIS2DH12_REG_INT1_SRC_ZL_MASK                   (1 << LIS2DH12_REG_INT1_SRC_ZL_BIT)
#define LIS2DH12_REG_INT1_SRC_YH_BIT                    (3)
#define LIS2DH12_REG_INT1_SRC_YH_MASK                   (1 << LIS2DH12_REG_INT1_SRC_YH_BIT)
#define LIS2DH12_REG_INT1_SRC_YL_BIT                    (2)
#define LIS2DH12_REG_INT1_SRC_YL_MASK                   (1 << LIS2DH12_REG_INT1_SRC_YL_BIT)
#define LIS2DH12_REG_INT1_SRC_XH_BIT                    (1)
#define LIS2DH12_REG_INT1_SRC_XH_MASK                   (1 << LIS2DH12_REG_INT1_SRC_XH_BIT)
#define LIS2DH12_REG_INT1_SRC_XL_BIT                    (0)
#define LIS2DH12_REG_INT1_SRC_XL_MASK                   (1 << LIS2DH12_REG_INT1_SRC_XL_BIT)

/**
 * @brief   Interrupt 1 threshold register
 */
#define LIS2DH12_REG_INT1_THS                           (0x32)
#define LIS2DH12_REG_INT1_THS_MASK                      (0x7F)

/**
 * @brief   Interrupt 1 duration register
 */
#define LIS2DH12_REG_INT1_DURATION                      (0x33)
#define LIS2DH12_REG_INT1_DURATION_MASK                 (0x7F)

/**
 * @brief   Interrupt 1 configuration register
 */
#define LIS2DH12_REG_INT2_CFG                           (0x34)
#define LIS2DH12_REG_INT2_CFG_AND_OR_BIT                (7)
#define LIS2DH12_REG_INT2_CFG_AND_OR_MASK               (1 << LIS2DH12_REG_INT2_CFG_AND_OR_BIT)
#define LIS2DH12_REG_INT2_CFG_INT_6D_BIT                (6)
#define LIS2DH12_REG_INT2_CFG_INT_6D_MASK               (1 << LIS2DH12_REG_INT2_CFG_INT_6D_BIT)
#define LIS2DH12_REG_INT2_CFG_ZHIE_BIT                  (5)
#define LIS2DH12_REG_INT2_CFG_ZHIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_ZHIE_BIT)
#define LIS2DH12_REG_INT2_CFG_ZLIE_BIT                  (4)
#define LIS2DH12_REG_INT2_CFG_ZLIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_ZLIE_BIT)
#define LIS2DH12_REG_INT2_CFG_YHIE_BIT                  (3)
#define LIS2DH12_REG_INT2_CFG_YHIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_YHIE_BIT)
#define LIS2DH12_REG_INT2_CFG_YLIE_BIT                  (2)
#define LIS2DH12_REG_INT2_CFG_YLIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_YLIE_BIT)
#define LIS2DH12_REG_INT2_CFG_XHIE_BIT                  (1)
#define LIS2DH12_REG_INT2_CFG_XHIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_XHIE_BIT)
#define LIS2DH12_REG_INT2_CFG_XLIE_BIT                  (0)
#define LIS2DH12_REG_INT2_CFG_XLIE_MASK                 (1 << LIS2DH12_REG_INT2_CFG_XLIE_BIT)

/**
 * @brief   Interrupt 1 source register
 */
#define LIS2DH12_REG_INT2_SRC                           (0x35)
#define LIS2DH12_REG_INT2_SRC_IA_BIT                    (6)
#define LIS2DH12_REG_INT2_SRC_IA_MASK                   (1 << LIS2DH12_REG_INT2_SRC_IA_BIT)
#define LIS2DH12_REG_INT2_SRC_ZH_BIT                    (5)
#define LIS2DH12_REG_INT2_SRC_ZH_MASK                   (1 << LIS2DH12_REG_INT2_SRC_ZH_BIT)
#define LIS2DH12_REG_INT2_SRC_ZL_BIT                    (4)
#define LIS2DH12_REG_INT2_SRC_ZL_MASK                   (1 << LIS2DH12_REG_INT2_SRC_ZL_BIT)
#define LIS2DH12_REG_INT2_SRC_YH_BIT                    (3)
#define LIS2DH12_REG_INT2_SRC_YH_MASK                   (1 << LIS2DH12_REG_INT2_SRC_YH_BIT)
#define LIS2DH12_REG_INT2_SRC_YL_BIT                    (2)
#define LIS2DH12_REG_INT2_SRC_YL_MASK                   (1 << LIS2DH12_REG_INT2_SRC_YL_BIT)
#define LIS2DH12_REG_INT2_SRC_XH_BIT                    (1)
#define LIS2DH12_REG_INT2_SRC_XH_MASK                   (1 << LIS2DH12_REG_INT2_SRC_XH_BIT)
#define LIS2DH12_REG_INT2_SRC_XL_BIT                    (0)
#define LIS2DH12_REG_INT2_SRC_XL_MASK                   (1 << LIS2DH12_REG_INT2_SRC_XL_BIT)

/**
 * @brief   Interrupt 1 threshold register
 */
#define LIS2DH12_REG_INT2_THS                           (0x36)
#define LIS2DH12_REG_INT2_THS_MASK                      (0x7F)

/**
 * @brief   Interrupt 1 duration register
 */
#define LIS2DH12_REG_INT2_DURATION                      (0x37)
#define LIS2DH12_REG_INT2_DURATION_MASK                 (0x7F)

/**
 * @brief   Click interrupt configuration register
 */
#define LIS2DH12_REG_CLICK_CFG                          (0x38)
#define LIS2DH12_REG_CLICK_CFG_ZD_BIT                   (5)
#define LIS2DH12_REG_CLICK_CFG_ZD_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_ZD_BIT)
#define LIS2DH12_REG_CLICK_CFG_ZS_BIT                   (4)
#define LIS2DH12_REG_CLICK_CFG_ZS_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_ZS_BIT)
#define LIS2DH12_REG_CLICK_CFG_YD_BIT                   (3)
#define LIS2DH12_REG_CLICK_CFG_YD_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_YD_BIT)
#define LIS2DH12_REG_CLICK_CFG_YS_BIT                   (2)
#define LIS2DH12_REG_CLICK_CFG_YS_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_YS_BIT)
#define LIS2DH12_REG_CLICK_CFG_XD_BIT                   (1)
#define LIS2DH12_REG_CLICK_CFG_XD_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_XD_BIT)
#define LIS2DH12_REG_CLICK_CFG_XS_BIT                   (0)
#define LIS2DH12_REG_CLICK_CFG_XS_MASK                  (1 << LIS2DH12_REG_CLICK_CFG_XS_BIT)

/**
 * @brief   Click interrupt source register
 */
#define LIS2DH12_REG_CLICK_SRC                          (0x39)
#define LIS2DH12_REG_CLICK_SRC_IA_BIT                   (6)
#define LIS2DH12_REG_CLICK_SRC_IA_MASK                  (1 << LIS2DH12_REG_CLICK_SRC_IA_BIT)
#define LIS2DH12_REG_CLICK_SRC_DCLICK_BIT               (5)
#define LIS2DH12_REG_CLICK_SRC_DCLICK_MASK              (1 << LIS2DH12_REG_CLICK_SRC_DCLICK_BIT)
#define LIS2DH12_REG_CLICK_SRC_SCLICK_BIT               (4)
#define LIS2DH12_REG_CLICK_SRC_SCLICK_MASK              (1 << LIS2DH12_REG_CLICK_SRC_SCLICK_BIT)
#define LIS2DH12_REG_CLICK_SRC_SIGN_BIT                 (3)
#define LIS2DH12_REG_CLICK_SRC_SIGN_MASK                (1 << LIS2DH12_REG_CLICK_SRC_SIGN_BIT)
#define LIS2DH12_REG_CLICK_SRC_Z_BIT                    (2)
#define LIS2DH12_REG_CLICK_SRC_Z_MASK                   (1 << LIS2DH12_REG_CLICK_SRC_Z_BIT)
#define LIS2DH12_REG_CLICK_SRC_Y_BIT                    (1)
#define LIS2DH12_REG_CLICK_SRC_Y_MASK                   (1 << LIS2DH12_REG_CLICK_SRC_Y_BIT)
#define LIS2DH12_REG_CLICK_SRC_X_BIT                    (0)
#define LIS2DH12_REG_CLICK_SRC_X_MASK                   (1 << LIS2DH12_REG_CLICK_SRC_X_BIT)

/**
 * @brief   Click interrupt treshold register
 */
#define LIS2DH12_REG_CLICK_THS                          (0x3A)
#define LIS2DH12_REG_CLICK_THS_LIR_CLICK_BIT            (7)
#define LIS2DH12_REG_CLICK_THS_LIR_CLICK_MASK           (1 << LIS2DH12_REG_CLICK_THS_LIR_CLICK_BIT)
#define LIS2DH12_REG_CLICK_THS_THS_MASK                 (0x7F)

/**
 * @brief   Click interrupt time limit register
 */
#define LIS2DH12_REG_TIME_LIMIT                         (0x3B)
#define LIS2DH12_REG_TIME_LIMIT_MASK                    (0x7F)

/**
 * @brief   Click interrupt time latency register
 */
#define LIS2DH12_REG_TIME_LATENCY                       (0x3C)
#define LIS2DH12_REG_TIME_LATENCY_MASK                  (0xFF)

/**
 * @brief   Click interrupt time window register
 */
#define LIS2DH12_REG_TIME_WINDOW                        (0x3D)
#define LIS2DH12_REG_TIME_WINDOW_MASK                   (0xFF)

/**
 * @brief   Activation threshold register
 */
#define LIS2DH12_REG_ACT_THS                            (0x3E)
#define LIS2DH12_REG_ACT_THS_MASK                       (0x7F)

/**
 * @brief   Activation duration register
 * 
 */
#define LIS2DH12_REG_ACT_DUR                            (0x3F)
#define LIS2DH12_REG_ACT_DUR_MASK                       (0xFF)
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_INTERNAL_H */
/** @} */
