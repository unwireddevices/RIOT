/*
 * Copyright (C) 2018 Freie Universität Berlin
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
 * @brief       Command definition for the LIS2DH12 accelerometer
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
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

/***********************************************************************************************************/




#define LIS2DH12_REG_CTRL_REG5               (0x24)
#define LIS2DH12_REG_CTRL_REG6               (0x25)
#define LIS2DH12_REG_REFERENCE               (0x26)
#define LIS2DH12_REG_STATUS_REG              (0x27)
#define LIS2DH12_REG_OUT_X_L                 (0x28)
#define LIS2DH12_REG_OUT_X_H                 (0x29)
#define LIS2DH12_REG_OUT_Y_L                 (0x2A)
#define LIS2DH12_REG_OUT_Y_H                 (0x2B)
#define LIS2DH12_REG_OUT_Z_L                 (0x2C)
#define LIS2DH12_REG_OUT_Z_H                 (0x2D)
#define LIS2DH12_REG_FIFO_CTRL_REG           (0x2E)
#define LIS2DH12_REG_FIFO_SRC_REG            (0x2F)
#define LIS2DH12_REG_INT1_CFG                (0x30)
#define LIS2DH12_REG_INT1_SRC                (0x31)
#define LIS2DH12_REG_INT1_THS                (0x32)
#define LIS2DH12_REG_INT1_DURATION           (0x33)
#define LIS2DH12_REG_INT2_CFG                (0x34)
#define LIS2DH12_REG_INT2_SRC                (0x35)
#define LIS2DH12_REG_INT2_THS                (0x36)
#define LIS2DH12_REG_INT2_DURATION           (0x37)
#define LIS2DH12_REG_CLICK_CFG               (0x38)
#define LIS2DH12_REG_CLICK_SRC               (0x39)
#define LIS2DH12_REG_CLICK_THS               (0x3A)
#define LIS2DH12_REG_TIME_LIMIT              (0x3B)
#define LIS2DH12_REG_TIME_LATENCY            (0x3C)
#define LIS2DH12_REG_TIME_WINDOW             (0x3D)
#define LIS2DH12_REG_ACT_THS                 (0x3E)
#define LIS2DH12_REG_ACT_DUR                 (0x3F)
/** @} */
//////////////////////////////////////////////////////////////////////////////////////////////////////////





/**
 * @brief   Control register 5
 */
#define LIS3DH_REG_CTRL_REG5                        (0x24)
#define LIS3DH_REG_CTRL_REG5_BOOT                   (1 << 7)
#define LIS3DH_REG_CTRL_REG5_FIFO_EN                (1 << 6)
#define LIS3DH_REG_CTRL_REG5_LIR_INT1               (1 << 3)
#define LIS3DH_REG_CTRL_REG5_D4D_INT1               (1 << 2)
#define LIS3DH_REG_CTRL_REG5_LIR_INT2               (1 << 1)
#define LIS3DH_REG_CTRL_REG5_D4D_INT2               (1 << 0)

/**
 * @brief   Control register 6
 */
#define LIS3DH_REG_CTRL_REG6                        (0x25)
#define LIS3DH_REG_CTRL_REG6_I2_CLICK               (1 << 7)
#define LIS3DH_REG_CTRL_REG6_I2_CLICK_SHIFT         (7)
#define LIS3DH_REG_CTRL_REG6_I2_IA1                 (1 << 6)
#define LIS3DH_REG_CTRL_REG6_I2_IA1_SHIFT           (6)
#define LIS3DH_REG_CTRL_REG6_I2_IA2                 (1 << 6)
#define LIS3DH_REG_CTRL_REG6_I2_IA2_SHIFT           (5)
#define LIS3DH_REG_CTRL_REG6_I2_BOOT                (1 << 4)
#define LIS3DH_REG_CTRL_REG6_I2_BOOT_SHIFT          (4)
#define LIS3DH_REG_CTRL_REG6_I2_ACT                 (1 << 3)
#define LIS3DH_REG_CTRL_REG6_I2_ACT_SHIFT           (3)
#define LIS3DH_REG_CTRL_REG6_INT_POLARITY           (1 << 1)


/**
 * @brief   Reference/datacapture register
 */
#define LIS3DH_REG_REFERENCE                        (0x26)
#define LIS3DH_REG_REFERENCE_REF                    (1 << 0)

/**
 * @brief   Axis status register
 */
#define LIS3DH_REG_STATUS_REG                       (0x27)
#define LIS3DH_REG_STATUS_REG_ZYXOR                 (1 << 7)
#define LIS3DH_REG_STATUS_REG_ZOR                   (1 << 6)
#define LIS3DH_REG_STATUS_REG_YOR                   (1 << 5)
#define LIS3DH_REG_STATUS_REG_XOR                   (1 << 4)
#define LIS3DH_REG_STATUS_REG_ZYXDA                 (1 << 3)
#define LIS3DH_REG_STATUS_REG_ZDA                   (1 << 2)
#define LIS3DH_REG_STATUS_REG_YDA                   (1 << 1)
#define LIS3DH_REG_STATUS_REG_XDA                   (1 << 0)

/**
 * @brief Acceleration data registers
 */
#define LIS3DH_REG_OUT_X_L                          (0x28)
#define LIS3DH_REG_OUT_X_H                          (0x29)
#define LIS3DH_REG_OUT_Y_L                          (0x2A)
#define LIS3DH_REG_OUT_Y_H                          (0x2B)
#define LIS3DH_REG_OUT_Z_L                          (0x2C)
#define LIS3DH_REG_OUT_Z_H                          (0x2D)


/**
 * @brief   FIFO control register
 */
#define LIS3DH_REG_FIFO_CTRL_REG                    (0x2E)
#define LIS3DH_REG_FIFO_CTRL_REG_FM_MASK            (0xC0)
#define LIS3DH_REG_FIFO_CTRL_REG_FM_SHIFT           (6)
#define LIS3DH_REG_FIFO_CTRL_REG_TR                 (1 << 5)
#define LIS3DH_REG_FIFO_CTRL_REG_FTH_MASK           (0x1F)
#define LIS3DH_REG_FIFO_CTRL_REG_FTH_SHIFT          (0)

/**
 * @brief FIFO source register
 */
#define LIS3DH_REG_FIFO_SRC_REG                     (0x2F)
#define LIS3DH_REG_FIFO_SRC_REG_WTM                 (1 << 7)
#define LIS3DH_REG_FIFO_SRC_REG_OVRN_FIFO           (1 << 6)
#define LIS3DH_REG_FIFO_SRC_REG_EMPTY               (1 << 5)
#define LIS3DH_REG_FIFO_SRC_REG_FSS_MASK            (0x1F)
#define LIS3DH_REG_FIFO_SRC_REG_FSS_SHIFT           (0)

/**
 * @brief   Interrupt 1 configuration register
 */
#define LIS3DH_REG_INT1_CFG                         (0x30)
#define LIS3DH_REG_INT1_CFG_AND_OR                  (1 << 7)
#define LIS3DH_REG_INT1_CFG_INT_6D                  (1 << 6)
#define LIS3DH_REG_INT1_CFG_ZHIE                    (1 << 5)
#define LIS3DH_REG_INT1_CFG_ZLIE                    (1 << 4)
#define LIS3DH_REG_INT1_CFG_YHIE                    (1 << 3)
#define LIS3DH_REG_INT1_CFG_YLIE                    (1 << 2)
#define LIS3DH_REG_INT1_CFG_XHIE                    (1 << 1)
#define LIS3DH_REG_INT1_CFG_XLIE                    (1 << 0)

/**
 * @brief   Interrupt 1 source register
 */
#define LIS3DH_REG_INT1_SOURCE                      (0x31)
#define LIS3DH_REG_INT1_SRC_IA                      (1 << 6)
#define LIS3DH_REG_INT1_SRC_ZH                      (1 << 5)
#define LIS3DH_REG_INT1_SRC_ZL                      (1 << 4)
#define LIS3DH_REG_INT1_SRC_YH                      (1 << 3)
#define LIS3DH_REG_INT1_SRC_YL                      (1 << 2)
#define LIS3DH_REG_INT1_SRC_XH                      (1 << 1)
#define LIS3DH_REG_INT1_SRC_XL                      (1 << 0)

/**
 * @brief   Interrupt 1 threshold register
 */
#define LIS3DH_REG_INT1_THS                         (0x32)

/**
 * @brief   Interrupt 1 duration register
 */
#define LIS3DH_REG_INT1_DURATION                    (0x33)

/**
 * @brief   Interrupt 2 configuration register
 */
#define LIS3DH_REG_INT2_CFG                         (0x34)
#define LIS3DH_REG_INT2_CFG_AND_OR                  (1 << 7)
#define LIS3DH_REG_INT2_CFG_INT_6D                  (1 << 6)
#define LIS3DH_REG_INT2_CFG_ZHIE                    (1 << 5)
#define LIS3DH_REG_INT2_CFG_ZLIE                    (1 << 4)
#define LIS3DH_REG_INT2_CFG_YHIE                    (1 << 3)
#define LIS3DH_REG_INT2_CFG_YLIE                    (1 << 2)
#define LIS3DH_REG_INT2_CFG_XHIE                    (1 << 1)
#define LIS3DH_REG_INT2_CFG_XLIE                    (1 << 0)

/**
 * @brief   Interrupt 2 source register
 */
#define LIS3DH_REG_INT2_SOURCE                      (0x35)
#define LIS3DH_REG_INT2_SRC_IA                      (1 << 6)
#define LIS3DH_REG_INT2_SRC_ZH                      (1 << 5)
#define LIS3DH_REG_INT2_SRC_ZL                      (1 << 4)
#define LIS3DH_REG_INT2_SRC_YH                      (1 << 3)
#define LIS3DH_REG_INT2_SRC_YL                      (1 << 2)
#define LIS3DH_REG_INT2_SRC_XH                      (1 << 1)
#define LIS3DH_REG_INT2_SRC_XL                      (1 << 0)

/**
 * @brief   Interrupt 2 threshold register
 */
#define LIS3DH_REG_INT2_THS                         (0x36)

/**
 * @brief   Interrupt 2 duration register
 */
#define LIS3DH_REG_INT2_DURATION                    (0x37)

/**
 * @brief   Click interrupt configuration register
 */
#define LIS3DH_REG_CLICK_CFG                        (0x38)
#define LIS3DH_REG_CLICK_CFG_ZD                     (1 << 5)
#define LIS3DH_REG_CLICK_CFG_ZS                     (1 << 4)
#define LIS3DH_REG_CLICK_CFG_YD                     (1 << 3)
#define LIS3DH_REG_CLICK_CFG_YS                     (1 << 2)
#define LIS3DH_REG_CLICK_CFG_XD                     (1 << 1)
#define LIS3DH_REG_CLICK_CFG_XS                     (1 << 0)

/**
 * @brief   Click interrupt source register
 */
#define LIS3DH_REG_CLICK_SRC                        (0x39)
#define LIS3DH_REG_CLICK_SRC_IA                     (1 << 6)
#define LIS3DH_REG_CLICK_SRC_DCLICK                 (1 << 5)
#define LIS3DH_REG_CLICK_SRC_SCLICK                 (1 << 4)
#define LIS3DH_REG_CLICK_SRC_SIGN                   (1 << 3)
#define LIS3DH_REG_CLICK_SRC_Z                      (1 << 2)
#define LIS3DH_REG_CLICK_SRC_Y                      (1 << 1)
#define LIS3DH_REG_CLICK_SRC_X                      (1 << 0)

/**
 * @brief   Click interrupt treshold register
 */
#define LIS3DH_REG_CLICK_THS                        (0x3A)
#define LIS3DH_REG_CLICK_THS_LIR_CLICK              (1 << 7)
#define LIS3DH_REG_CLICK_THS_THS_MASK               (0x7F)

/**
 * @brief   Click interrupt time limit register
 */
#define LIS3DH_REG_TIME_LIMIT                       (0x3B)

/**
 * @brief   Click interrupt time latency register
 */
#define LIS3DH_REG_TIME_LATENCY                     (0x3C)

/**
 * @brief   Click interrupt time window register
 */
#define LIS3DH_REG_TIME_WINDOW                      (0x3D)

/**
 * @brief   Activation threshold register
 */
#define LIS3DH_REG_ACT_THS                          (0x3E)

/**
 * @brief   Activation duration register
 * 
 */
#define LIS3DH_REG_ACT_DUR                         (0x3F)
/** @} */


#ifdef __cplusplus
}
#endif

#endif /* LIS2DH12_INTERNAL_H */
/** @} */
