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

#ifndef LIS3DH_INTERNAL_H
#define LIS3DH_INTERNAL_H


#ifdef __cplusplus
extern "C" {
#endif


#define PROPERTY_DISABLE                            (0x00)
#define PROPERTY_ENABLE                             (0x01)

/**
 * @name    LIS3DH hardware register addresses
 * @{
 */
#define LIS3DH_REG_STATUS_AUX                       (0x07)
#define LIS3DH_REG_OUT_AUX_ADC1_L                   (0x08)
#define LIS3DH_REG_OUT_AUX_ADC1_H                   (0x09)
#define LIS3DH_REG_OUT_AUX_ADC2_L                   (0x0A)
#define LIS3DH_REG_OUT_AUX_ADC2_H                   (0x0B)
#define LIS3DH_REG_OUT_AUX_ADC3_L                   (0x0C)
#define LIS3DH_REG_OUT_AUX_ADC3_H                   (0x0D)

#define LIS3DH_REG_WHO_AM_I                         (0x0F)

#define LIS3DH_REG_CTRL_REG0                        (0x1E)
#define LIS3DH_REG_TEMP_CFG_REG                     (0x1F)
#define LIS3DH_REG_CTRL_REG1                        (0x20)
#define LIS3DH_REG_CTRL_REG2                        (0x21)
#define LIS3DH_REG_CTRL_REG3                        (0x22)
#define LIS3DH_REG_CTRL_REG4                        (0x23)
#define LIS3DH_REG_CTRL_REG5                        (0x24)
#define LIS3DH_REG_CTRL_REG6                        (0x25)
#define LIS3DH_REG_REFERENCE                        (0x26)
#define LIS3DH_REG_STATUS_REG                       (0x27)
#define LIS3DH_REG_OUT_X_L                          (0x28)
#define LIS3DH_REG_OUT_X_H                          (0x29)
#define LIS3DH_REG_OUT_Y_L                          (0x2A)
#define LIS3DH_REG_OUT_Y_H                          (0x2B)
#define LIS3DH_REG_OUT_Z_L                          (0x2C)
#define LIS3DH_REG_OUT_Z_H                          (0x2D)
#define LIS3DH_REG_FIFO_CTRL_REG                    (0x2E)
#define LIS3DH_REG_FIFO_SRC_REG                     (0x2F)
#define LIS3DH_REG_INT1_CFG                         (0x30)
#define LIS3DH_REG_INT1_SOURCE                      (0x31)
#define LIS3DH_REG_INT1_THS                         (0x32)
#define LIS3DH_REG_INT1_DURATION                    (0x33)
#define LIS3DH_REG_INT2_CFG                         (0x34)
#define LIS3DH_REG_INT2_SOURCE                      (0x35)
#define LIS3DH_REG_INT2_THS                         (0x36)
#define LIS3DH_REG_INT2_DURATION                    (0x37)     
#define LIS3DH_REG_CLICK_CFG                        (0x38)
#define LIS3DH_REG_CLICK_SRC                        (0x39)
#define LIS3DH_REG_CLICK_THS                        (0x3A)
#define LIS3DH_REG_TIME_LIMIT                       (0x3B)
#define LIS3DH_REG_TIME_LATENCY                     (0x3C)
#define LIS3DH_REG_TIME_WINDOW                      (0x3D)
#define LIS3DH_REG_ACT_THS                          (0x3E)
#define LIS3DH_REG_IACT_DUR                         (0x3F)
/** @} */

/**
 * @name    LIS3DH hardware register structures
 * @{
 */

typedef struct {
    uint8_t _1da            : 1;
    uint8_t _2da            : 1;
    uint8_t _3da            : 1;
    uint8_t _321da          : 1;
    uint8_t _1or            : 1;
    uint8_t _2or            : 1;
    uint8_t _3or            : 1;
    uint8_t _321or          : 1;
} lis3dh_status_reg_aux_t;


typedef struct {
    uint8_t reserved        : 7;
    uint8_t sdo_pu_disc     : 1;
} lis3dh_ctrl_reg0_t;

typedef struct {
    uint8_t reserved        : 6;
    uint8_t adc_pd          : 1;
    uint8_t temp_en         : 1;
} lis3dh_temp_cfg_reg_t;


typedef struct {
    uint8_t xen             : 1;
    uint8_t yen             : 1;
    uint8_t zen             : 1;
    uint8_t lpen            : 1;
    uint8_t odr             : 4;
} lis3dh_ctrl_reg1_t;


typedef struct {
    uint8_t hp              : 3; /* HPCLICK + HP_IA2 + HP_IA1 -> HP */
    uint8_t fds             : 1;
    uint8_t hpcf            : 2;
    uint8_t hpm             : 2;
} lis3dh_ctrl_reg2_t;

typedef struct {
    uint8_t reserved_01     : 1;
    uint8_t i1_overrun      : 1;
    uint8_t i1_wtm          : 1;
    uint8_t reserved_02     : 1;
    uint8_t i1_zyxda        : 1;
    uint8_t i1_ia2          : 1;
    uint8_t i1_ia1          : 1;
    uint8_t i1_click        : 1;
} lis3dh_ctrl_reg3_t;

typedef struct {
    uint8_t sim             : 1;
    uint8_t st              : 2;
    uint8_t hr              : 1;
    uint8_t fs              : 2;
    uint8_t ble             : 1;
    uint8_t bdu             : 1;
} lis3dh_ctrl_reg4_t;

typedef struct {
    uint8_t d4d_int2        : 1;
    uint8_t lir_int2        : 1;
    uint8_t d4d_int1        : 1;
    uint8_t lir_int1        : 1;
    uint8_t reserved        : 2;
    uint8_t fifo_en         : 1;
    uint8_t boot            : 1;
} lis3dh_ctrl_reg5_t;

typedef struct {
    uint8_t reserved_01     : 1;
    uint8_t int_polarity    : 1;
    uint8_t reserved_02     : 1;
    uint8_t i2_act          : 1;
    uint8_t i2_boot         : 1;
    uint8_t i2_ia2          : 1;
    uint8_t i2_ia1          : 1;
    uint8_t i2_click        : 1;
} lis3dh_ctrl_reg6_t;

typedef struct {
    uint8_t _xda            : 1;
    uint8_t _yda            : 1;
    uint8_t _zda            : 1;
    uint8_t _zyxda          : 1;
    uint8_t _xor            : 1;
    uint8_t _yor            : 1;
    uint8_t _zor            : 1;
    uint8_t _zyxor          : 1;
} lis3dh_status_reg_t;

typedef struct {
    uint8_t fth             : 5;
    uint8_t tr              : 1;
    uint8_t fm              : 2;
} lis3dh_fifo_ctrl_reg_t;

typedef struct {
    uint8_t fss             : 5;
    uint8_t empty           : 1;
    uint8_t ovrn_fifo       : 1;
    uint8_t wtm             : 1;
} lis3dh_fifo_src_reg_t;

typedef struct {
    uint8_t xlie            : 1;
    uint8_t xhie            : 1;
    uint8_t ylie            : 1;
    uint8_t yhie            : 1;
    uint8_t zlie            : 1;
    uint8_t zhie            : 1;
    uint8_t _6d             : 1;
    uint8_t aoi             : 1;
} lis3dh_int1_cfg_t;


typedef struct {
    uint8_t xl              : 1;
    uint8_t xh              : 1;
    uint8_t yl              : 1;
    uint8_t yh              : 1;
    uint8_t zl              : 1;
    uint8_t zh              : 1;
    uint8_t ia              : 1;
    uint8_t reserved_01     : 1;
} lis3dh_int1_src_t;


typedef struct {
    uint8_t ths             : 7;
    uint8_t reserved        : 1;
} lis3dh_int1_ths_t;

typedef struct {
    uint8_t d               : 7;
    uint8_t reserved        : 1;
} lis3dh_int1_duration_t;

typedef struct {
    uint8_t xlie            : 1;
    uint8_t xhie            : 1;
    uint8_t ylie            : 1;
    uint8_t yhie            : 1;
    uint8_t zlie            : 1;
    uint8_t zhie            : 1;
    uint8_t _6d             : 1;
    uint8_t aoi             : 1;
} lis3dh_int2_cfg_t;


typedef struct {
    uint8_t xl              : 1;
    uint8_t xh              : 1;
    uint8_t yl              : 1;
    uint8_t yh              : 1;
    uint8_t zl              : 1;
    uint8_t zh              : 1;
    uint8_t ia              : 1;
    uint8_t reserved_01     : 1;
} lis3dh_int2_src_t;



typedef struct {
    uint8_t ths             : 7;
    uint8_t reserved_01     : 1;
} lis3dh_int2_ths_t;


typedef struct {
    uint8_t d               : 7;
    uint8_t reserved_01     : 1;
} lis3dh_int2_duration_t;

typedef struct {
    uint8_t xs              : 1;
    uint8_t xd              : 1;
    uint8_t ys              : 1;
    uint8_t yd              : 1;
    uint8_t zs              : 1;
    uint8_t zd              : 1;
    uint8_t reserved_01     : 2;
} lis3dh_click_cfg_t;

typedef struct {
    uint8_t x               : 1;
    uint8_t y               : 1;
    uint8_t z               : 1;
    uint8_t sign            : 1;
    uint8_t sclick          : 1;
    uint8_t dclick          : 1;
    uint8_t ia              : 1;
    uint8_t reserved_01     : 1;
} lis3dh_click_src_t;

typedef struct {
    uint8_t ths             : 7;
    uint8_t lir_click       : 1;
} lis3dh_click_ths_t;

typedef struct {
    uint8_t tli             : 7;
    uint8_t reserved_01     : 1;
} lis3dh_time_limit_t;

typedef struct {
    uint8_t tla             : 8;
} lis3dh_time_latency_t;

typedef struct {
    uint8_t tw              : 8;
} lis3dh_time_window_t;

typedef struct {
    uint8_t acth            : 7;
    uint8_t reserved_01     : 1;
} lis3dh_act_ths_t;

typedef struct {
    uint8_t actd            : 8;
} lis3dh_act_dur_t;
/** @} */

/**
 * @name    LIS3DH union any structure registers and Bytes
 */
typedef struct{
    uint8_t bit0 : 1;
    uint8_t bit1 : 1;
    uint8_t bit2 : 1;
    uint8_t bit3 : 1;
    uint8_t bit4 : 1;
    uint8_t bit5 : 1;
    uint8_t bit6 : 1;
    uint8_t bit7 : 1;
} bitwise_t;

typedef union{
    lis3dh_status_reg_aux_t status_reg_aux;
    lis3dh_ctrl_reg0_t      ctrl_reg0;
    lis3dh_temp_cfg_reg_t   temp_cfg_reg;
    lis3dh_ctrl_reg1_t      ctrl_reg1;
    lis3dh_ctrl_reg2_t      ctrl_reg2;
    lis3dh_ctrl_reg3_t      ctrl_reg3;
    lis3dh_ctrl_reg4_t      ctrl_reg4;
    lis3dh_ctrl_reg5_t      ctrl_reg5;
    lis3dh_ctrl_reg6_t      ctrl_reg6;
    lis3dh_status_reg_t     status_reg;
    lis3dh_fifo_ctrl_reg_t  fifo_ctrl_reg;
    lis3dh_fifo_src_reg_t   fifo_src_reg;
    lis3dh_int1_cfg_t       int1_cfg;
    lis3dh_int1_src_t       int1_src;
    lis3dh_int1_ths_t       int1_ths;
    lis3dh_int1_duration_t  int1_duration;
    lis3dh_int2_cfg_t       int2_cfg;
    lis3dh_int2_src_t       int2_src;
    lis3dh_int2_ths_t       int2_ths;
    lis3dh_int2_duration_t  int2_duration;
    lis3dh_click_cfg_t      click_cfg;
    lis3dh_click_src_t      click_src;
    lis3dh_click_ths_t      click_ths;
    lis3dh_time_limit_t     time_limit;
    lis3dh_time_latency_t   time_latency;
    lis3dh_time_window_t    time_window;
    lis3dh_act_ths_t        act_ths;
    lis3dh_act_dur_t        act_dur;
    bitwise_t               bitwise;
    uint8_t                 byte;
} lis3dh_reg_t;

/**
 * @name    LIS3DH values registers enumirations
 * @{ 
 */
typedef enum {
  LIS3DH_AUX_DISABLE          = 0,
  LIS3DH_AUX_ON_TEMPERATURE   = 3,
  LIS3DH_AUX_ON_PADS          = 1,
} lis3dh_temp_en_t;

typedef enum {
  LIS3DH_HR_12bit   = 0,
  LIS3DH_NM_10bit   = 1,
  LIS3DH_LP_8bit    = 2,
} lis3dh_op_md_t;

typedef enum {
  LIS3DH_POWER_DOWN                      = 0x00,
  LIS3DH_ODR_1Hz                         = 0x01,
  LIS3DH_ODR_10Hz                        = 0x02,
  LIS3DH_ODR_25Hz                        = 0x03,
  LIS3DH_ODR_50Hz                        = 0x04,
  LIS3DH_ODR_100Hz                       = 0x05,
  LIS3DH_ODR_200Hz                       = 0x06,
  LIS3DH_ODR_400Hz                       = 0x07,
  LIS3DH_ODR_1kHz620_LP                  = 0x08,
  LIS3DH_ODR_5kHz376_LP_1kHz344_NM_HP    = 0x09,
} lis3dh_odr_t;

typedef enum {
  LIS3DH_AGGRESSIVE  = 0,
  LIS3DH_STRONG      = 1,
  LIS3DH_MEDIUM      = 2,
  LIS3DH_LIGHT       = 3,
} lis3dh_hpcf_t;

typedef enum {
  LIS3DH_NORMAL_WITH_RST  = 0,
  LIS3DH_REFERENCE_MODE   = 1,
  LIS3DH_NORMAL           = 2,
  LIS3DH_AUTORST_ON_INT   = 3,
} lis3dh_hpm_t;

typedef enum {
  LIS3DH_2g   = 0,
  LIS3DH_4g   = 1,
  LIS3DH_8g   = 2,
  LIS3DH_16g  = 3,
} lis3dh_fs_t;

typedef enum {
  LIS3DH_ST_DISABLE   = 0,
  LIS3DH_ST_POSITIVE  = 1,
  LIS3DH_ST_NEGATIVE  = 2,
} lis3dh_st_t;

typedef enum {
  LIS3DH_LSB_AT_LOW_ADD = 0,
  LIS3DH_MSB_AT_LOW_ADD = 1,
} lis3dh_ble_t;

typedef enum {
  LIS3DH_DISC_FROM_INT_GENERATOR  = 0,
  LIS3DH_ON_INT1_GEN              = 1,
  LIS3DH_ON_INT2_GEN              = 2,
  LIS3DH_ON_TAP_GEN               = 4,
  LIS3DH_ON_INT1_INT2_GEN         = 3,
  LIS3DH_ON_INT1_TAP_GEN          = 5,
  LIS3DH_ON_INT2_TAP_GEN          = 6,
  LIS3DH_ON_INT1_INT2_TAP_GEN     = 7,
} lis3dh_hp_t;

typedef enum {
  LIS3DH_INT2_PULSED   = 0,
  LIS3DH_INT2_LATCHED  = 1,
} lis3dh_lir_int2_t;

typedef enum {
  LIS3DH_INT1_PULSED   = 0,
  LIS3DH_INT1_LATCHED  = 1,
} lis3dh_lir_int1_t;

typedef enum {
  LIS3DH_INT1_GEN = 0,
  LIS3DH_INT2_GEN = 1,
} lis3dh_tr_t;

typedef enum {
  LIS3DH_BYPASS_MODE           = 0,
  LIS3DH_FIFO_MODE             = 1,
  LIS3DH_DYNAMIC_STREAM_MODE   = 2,
  LIS3DH_STREAM_TO_FIFO_MODE   = 3,
} lis3dh_fm_t;

typedef enum {
  LIS3DH_TAP_PULSED   = 0,
  LIS3DH_TAP_LATCHED  = 1,
} lis3dh_lir_click_t;

typedef enum {
  LIS3DH_PULL_UP_DISCONNECT  = 0,
  LIS3DH_PULL_UP_CONNECT     = 1,
} lis3dh_sdo_pu_disc_t;

typedef enum {
  LIS3DH_SPI_4_WIRE = 0,
  LIS3DH_SPI_3_WIRE = 1,
} lis3dh_sim_t;
/** @} */


/**
 * @brief   Identifier register value
 *
 * The WHO_AM_I register should contain this value in order to correctly
 * identify the chip.
 */
#define LIS3DH_WHO_AM_I_RESPONSE                    (0x33)

/*
 * Bit offsets within the individual registers
 * source: LIS3DH datasheet
 */

/**
 * @name    TEMP_CFG_REG bitfield macros
 * @{
 */
/**
 * @brief   ADC enable
 *
 * Default value: 0
 *
 * 0: ADC disabled; 1: ADC enabled
 */
#define LIS3DH_TEMP_CFG_REG_ADC_PD_MASK             (1 << 7)
/**
 * @brief ADC enable bit offset
 * 
 */
#define LIS3DH_TEMP_CFG_REG_ADC_PD_SHIFT            (7)    
/**
 * @brief   Temperature sensor (T) enable.
 *
 * Default value: 0
 *
 * 0: T disabled; 1: T enabled
 */
#define LIS3DH_TEMP_CFG_REG_TEMP_EN_MASK            (1 << 6)
/**
 * @brief Temperature sensor (T) enable bit offset
 * 
 */
#define LIS3DH_TEMP_CFG_REG_TEMP_EN_SHIFT           (6)    

/** @} */ /* TEMP_CFG_REG bitfield macros */


/**
 * @name    CTRL_REG0 bitfield macros
 * @{
 */
/**
 * @brief  Disconnect SDO/SA0 pull-up.
 *
 * Default value: 0
 *
 * 0: PU connected; 1: PU disconnected
 */     
#define LIS3DH_CTRL_REG0_SDO_PU_DISC_MASK           (1 << 7)
/**
 * @brief    enable SDO pin pull-up
 */
#define LIS3DH_CTRL_REG0_SDO_PU_DISABLE             (LIS3DH_CTRL_REG0_SDO_PU_DISC_MASK)

/** @} */  /* CTRL_REG0 bitfield macros */

/**
 * @name    CTRL_REG1 bitfield macros
 * @{
 */
/**
 * @brief    ODR global shift
 */
#define LIS3DH_CTRL_REG1_ODR_SHIFT                  (4)
/**
 * @brief    ODR fourth bit mask
 */
#define LIS3DH_CTRL_REG1_ODR3_MASK                  (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 3))
/**
 * @brief    ODR third bit mask
 */
#define LIS3DH_CTRL_REG1_ODR2_MASK                  (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 2))
/**
 * @brief    ODR second bit mask
 */
#define LIS3DH_CTRL_REG1_ODR1_MASK                  (1 << (LIS3DH_CTRL_REG1_ODR_SHIFT + 1))
/**
 * @brief   ODR first bit mask
 */
#define LIS3DH_CTRL_REG1_ODR0_MASK                  (1 << LIS3DH_CTRL_REG1_ODR_SHIFT)
/**
 * @brief   Output data rate (ODR) selection bitfield
 *
 * Default value: 0000
 *
 * 0000: Power down; Others: Refer to data sheet
 *
 * @see LIS3DH data sheet Table 25, “Data rate configuration”
 */
#define LIS3DH_CTRL_REG1_ODR_MASK                   (LIS3DH_CTRL_REG1_ODR3_MASK | \
                                                     LIS3DH_CTRL_REG1_ODR2_MASK | \
                                                     LIS3DH_CTRL_REG1_ODR1_MASK | \
                                                     LIS3DH_CTRL_REG1_ODR0_MASK)
/**
 * @brief   Low power mode enable.
 *
 * Default value: 0
 *
 *  0. normal mode
 *  1. low power mode
 */
#define LIS3DH_CTRL_REG1_LPEN_MASK                  (1 << 3)
/**
 * @brief   Z enable bit offset
 */
#define LIS3DH_CTRL_REG1_ZEN_SHIFT                  (2)
/**
 * @brief   Z axis enable.
 *
 * Default value: 1
 *
 *  0. Z axis disabled
 *  1. Z axis enabled
 */
#define LIS3DH_CTRL_REG1_ZEN_MASK                   (1 << LIS3DH_CTRL_REG1_ZEN_SHIFT)
/**
 * @brief   Y enable bit offset
 */
#define LIS3DH_CTRL_REG1_YEN_SHIFT                  (1)
/**
 * @brief   Y axis enable.
 *
 * Default value: 1
 *
 *  0. Y axis disabled
 *  1. Y axis enabled
 */
#define LIS3DH_CTRL_REG1_YEN_MASK                   (1 << LIS3DH_CTRL_REG1_YEN_SHIFT)
/**
 * @brief   X enable bit offset
 */
#define LIS3DH_CTRL_REG1_XEN_SHIFT                  (0)
/**
 * @brief   X axis enable.
 *
 * Default value: 1
 *
 *  0. X axis disabled
 *  1. X axis enabled
 */
#define LIS3DH_CTRL_REG1_XEN_MASK                   (1 << LIS3DH_CTRL_REG1_XEN_SHIFT)
/**
 * @brief   XYZ enable bitfield offset
 */
#define LIS3DH_CTRL_REG1_XYZEN_SHIFT                (0)
/**
 * @brief   X, Y, Z enable bitfield mask
 */
#define LIS3DH_CTRL_REG1_XYZEN_MASK                 (LIS3DH_CTRL_REG1_XEN_MASK | \
                                                     LIS3DH_CTRL_REG1_YEN_MASK | \
                                                     LIS3DH_CTRL_REG1_ZEN_MASK)

/**
 * @brief    enable X axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_X                               (LIS3DH_CTRL_REG1_XEN_MASK)
/**
 * @brief   enable Y axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_Y                               (LIS3DH_CTRL_REG1_YEN_MASK)
/**
 * @brief   enable Z axis (Use when calling lis3dh_set_axes())
 */
#define LIS3DH_AXES_Z                               (LIS3DH_CTRL_REG1_ZEN_MASK)
/** @} */  /* CTRL_REG1 bitfield macros */

/**
 * @brief   Convenience macro for enabling all axes.
 */
#define LIS3DH_AXES_XYZ                             (LIS3DH_CTRL_REG1_XYZEN_MASK)

/**
 * @name    CTRL_REG2 bitfield macros
 * @{
 */
/**
 * @brief   High pass filter mode selection second bit
 *
 * Default value: 0
 *
 * @see Refer to Table 29, "High pass filter mode configuration"
 */
#define LIS3DH_CTRL_REG2_HPM1_MASK                  (1 << 7)
/**
 * @brief   High pass filter mode selection first bit
 *
 * Default value: 0
 *
 * @see Refer to Table 29, "High pass filter mode configuration"
 */
#define LIS3DH_CTRL_REG2_HPM0_MASK                  (1 << 6)
/**
 * @brief   High pass filter cut off frequency selection second bit
 */
#define LIS3DH_CTRL_REG2_HPCF2_MASK                 (1 << 5)
/**
 * @brief   High pass filter cut off frequency selection second bit
 */
#define LIS3DH_CTRL_REG2_HPCF1_MASK                 (1 << 4)
/**
 * @brief   Filtered data selection
 *
 * Default value: 0
 *
 *  0. internal filter bypassed
 *  1. data from internal filter sent to output register and FIFO
 */
#define LIS3DH_CTRL_REG2_FDS_MASK                   (1 << 3)
/**
 * @brief   High pass filter enabled for CLICK function.
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPCLICK_MASK               (1 << 2)
/**
 * @brief   High pass filter enabled for AOI function on interrupt 2, second bit
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPIS2_MASK                 (1 << 1)
/**
 * @brief   High pass filter enabled for AOI function on interrupt 2, first bit
 *
 *  0. filter bypassed
 *  1. filter enabled
 */
#define LIS3DH_CTRL_REG2_HPIS1_MASK                 (1 << 0)
/** @} */ /* CTRL_REG2 bitfield macros */

/**
 * @name    CTRL_REG3 bitfield macros
 * @{
 */
/**
 * @brief   CLICK interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_CLICK_MASK              (1 << 7)
/**
 * @brief   AOI1 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_AOI1_MASK               (1 << 6)
/**
 * @brief   AOI2 interrupt on INT1.
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_AOI2_MASK               (1 << 5)
/**
 * @brief   DRDY1 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_DRDY1_MASK              (1 << 4)
/**
 * @brief   DRDY2 interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_DRDY2_MASK              (1 << 3)
/**
 * @brief   FIFO Watermark interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_WTM_MASK                (1 << 2)
/**
 * @brief   FIFO Overrun interrupt on INT1
 *
 * Default value 0.
 *
 *  0. Disable
 *  1. Enable
 */
#define LIS3DH_CTRL_REG3_I1_OVERRUN_MASK            (1 << 1)
/** @} */ /* CTRL_REG3 bitfield macros */

/**
 * @name CTRL_REG4 bitfield macros
 * @{
 */
/**
 * @brief   Block data update (BDU) bit mask
 *
 * Default value of BDU: 0
 *
 *  0. continuous update
 *  1. output registers not updated until MSB and LSB reading
 */
#define LIS3DH_CTRL_REG4_BDU_MASK                   (1 << 7)
/**
 * @brief   Block data update (BDU) bitfield offset
 */
#define LIS3DH_CTRL_REG4_BDU_SHIFT                (7)

/**
 * @brief   Block data update (BDU) enable
 */
#define LIS3DH_CTRL_REG4_BDU_ENABLE                 (LIS3DH_CTRL_REG4_BDU_MASK)
/**
 * @brief    Block data update (BDU) disable
 */
#define LIS3DH_CTRL_REG4_BDU_DISABLE                (0)
/**
 * @brief   Big/little endian bit mask
 *
 * Default value of BLE: 0.
 *
 *  0. Data LSB @ lower address
 *  1. Data MSB @ lower address
 */
#define LIS3DH_CTRL_REG4_BLE_MASK                   (1 << 6)
/**
 * @brief   Big/little endian little endian mode
 */
#define LIS3DH_CTRL_REG4_BLE_LITTLE_ENDIAN          (0)
/**
 * @brief   Big/little endian big endian mode
 */
#define LIS3DH_CTRL_REG4_BLE_BIG_ENDIAN             (LIS3DH_CTRL_REG4_BLE_MASK)
/**
 * @brief   Full scale selection mask second bit
 */
#define LIS3DH_CTRL_REG4_FS1_MASK                   (1 << 5)
/**
 * @brief   Full scale selection mask first bit
 */
#define LIS3DH_CTRL_REG4_FS0_MASK                   (1 << 4)
/**
 * @brief   Full scale selection mask
 */
#define LIS3DH_CTRL_REG4_FS_MASK                    (LIS3DH_CTRL_REG4_FS1_MASK | \
                                                     LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   Scale register value: +/- 2G
 */
#define LIS3DH_CTRL_REG4_SCALE_2G                   (0)
/**
 * @brief   Scale register value: +/- 4G
 */
#define LIS3DH_CTRL_REG4_SCALE_4G                   (LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   Scale register value: +/- 8G
 */
#define LIS3DH_CTRL_REG4_SCALE_8G                   (LIS3DH_CTRL_REG4_FS1_MASK)
/**
 * @brief   Scale: +/- 16G
 */
#define LIS3DH_CTRL_REG4_SCALE_16G                  (LIS3DH_CTRL_REG4_FS1_MASK | \
                                                     LIS3DH_CTRL_REG4_FS0_MASK)
/**
 * @brief   High resolution output mode
 *
 * Default value: 0
 *
 *  0. High resolution disable
 *  1. High resolution enable
 */
#define LIS3DH_CTRL_REG4_HR_MASK                    (1 << 3)
/**
 * @brief   Self test enable second bit mask
 *
 * Default value of self test: 00
 *
 *  - 00: Self test disabled
 *  - Other: See Table 34
 *
 * @see Table 34
 */
#define LIS3DH_CTRL_REG4_ST1_MASK                   (1 << 2)
/**
 * @brief   Self test enable first bit mask
 */
#define LIS3DH_CTRL_REG4_ST0_MASK                   (1 << 1)
/**
 * @brief   SPI serial interface mode selection
 *
 * Default value: 0
 *
 *  0. 4-wire interface
 *  1. 3-wire interface
 */
#define LIS3DH_CTRL_REG4_SIM_MASK                   (1 << 0)
/**
 * @brief   Reboot memory content
 *
 * Default value: 0
 *
 *  0. normal mode
 *  1. reboot memory content
 */
#define LIS3DH_CTRL_REG5_REBOOT_MASK                (1 << 7)
/**
 * @brief   FIFO enable
 *
 * Default value: 0
 *
 *  0. FIFO disable
 *  1. FIFO enable
 */
#define LIS3DH_CTRL_REG5_FIFO_EN_MASK               (1 << 6)
/**
 * @brief   Latch interrupt request on INT1
 *
 * Latch interrupt request on INT1_SRC register, with INT1_SRC register
 * cleared by reading INT1_SRC itself.
 *
 * Default value: 0
 *
 *  0. interrupt request not latched
 *  1. interrupt request latched
 */
#define LIS3DH_CTRL_REG5_LIR_I1_MASK                (1 << 3)
/**
 * @brief   4D enable
 *
 * 4D detection is enabled on INT1 when 6D bit on INT1_CFG is set to 1.
 */
#define LIS3DH_CTRL_REG5_D4D_I1_MASK                (1 << 2)
/**
 * @brief   Latch interrupt request on INT2
 *
 * Latch interrupt request on INT2_SRC register, with INT2_SRC register
 * cleared by reading INT2_SRC itself.
 *
 * Default value: 0
 *
 *  0. interrupt request not latched
 *  1. interrupt request latched
 */
#define LIS3DH_CTRL_REG5_LIR_I2_MASK                (1 << 1)
/**
 * @brief   4D enable
 *
 * 4D detection is enabled on INT2 when 6D bit on INT2_CFG is set to 1.
 */
#define LIS3DH_CTRL_REG5_D4D_I2_MASK                (1 << 0)  
/** @} */ /* CTRL_REG4 bitfield macros */

/**
 * @name    STATUS_REG bitfield macros
 * @{
 */
/**
 * @brief   X, Y or Z axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new set of data has overwritten the previous ones
 */
#define LIS3DH_STATUS_REG_ZYXOR_MASK                (1 << 7)
/**
 * @brief   Z axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the Z-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_ZOR_MASK                  (1 << 6)
/**
 * @brief   Y axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the Y-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_YOR_MASK                  (1 << 5)
/**
 * @brief   X axis data overrun
 *
 * Default value: 0
 *
 *  0. no overrun has occurred
 *  1. a new data for the X-axis has overwritten the previous one
 */
#define LIS3DH_STATUS_REG_XOR_MASK                  (1 << 4)
/**
 * @brief   X, Y or Z axis new data available
 *
 * Default value: 0
 *
 *  0. a new set of data is not yet available
 *  1. a new set of data is available
 */
#define LIS3DH_STATUS_REG_ZYXDA_MASK                (1 << 3)
/**
 * @brief   Z axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the Z-axis is not yet available
 *  1. a new data for the Z-axis is available
 */
#define LIS3DH_STATUS_REG_ZDA_MASK                  (1 << 2)
/**
 * @brief   Y axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the Y-axis is not yet available
 *  1. a new data for the Y-axis is available
 */
#define LIS3DH_STATUS_REG_YDA_MASK                  (1 << 1)
/**
 * @brief   X axis new data available
 *
 * Default value: 0
 *
 *  0. a new data for the X-axis is not yet available
 *  1. a new data for the X-axis is available
 */
#define LIS3DH_STATUS_REG_XDA_MASK                  (1 << 0)
/** @} */ /* STATUS_REG bitfield macros */

/**
 * @name    FIFO_CTRL_REG bitfield macros
 * @{
 */
#define LIS3DH_FIFO_CTRL_REG_FM_SHIFT               (6)
#define LIS3DH_FIFO_CTRL_REG_FM1_MASK               (1 << 7)
#define LIS3DH_FIFO_CTRL_REG_FM0_MASK               (1 << 6)
#define LIS3DH_FIFO_CTRL_REG_FM_MASK                (LIS3DH_FIFO_CTRL_REG_FM1_MASK | \
                                                     LIS3DH_FIFO_CTRL_REG_FM0_MASK)
#define LIS3DH_FIFO_CTRL_REG_TR_MASK                (1 << 5)
#define LIS3DH_FIFO_CTRL_REG_FTH4_MASK              (1 << 4)
#define LIS3DH_FIFO_CTRL_REG_FTH3_MASK              (1 << 3)
#define LIS3DH_FIFO_CTRL_REG_FTH2_MASK              (1 << 2)
#define LIS3DH_FIFO_CTRL_REG_FTH1_MASK              (1 << 1)
#define LIS3DH_FIFO_CTRL_REG_FTH0_MASK              (1 << 0)
#define LIS3DH_FIFO_CTRL_REG_FTH_SHIFT              (0)
#define LIS3DH_FIFO_CTRL_REG_FTH_MASK               (LIS3DH_FIFO_CTRL_REG_FTH0_MASK | \
                                                     LIS3DH_FIFO_CTRL_REG_FTH1_MASK | \
                                                     LIS3DH_FIFO_CTRL_REG_FTH2_MASK | \
                                                     LIS3DH_FIFO_CTRL_REG_FTH3_MASK | \
                                                     LIS3DH_FIFO_CTRL_REG_FTH4_MASK)
/** @} */ /* FIFO_CTRL_REG bitfield macros */

/**
 * @name    FIFO_SRC_REG bitfield macros
 * @{
 */
#define LIS3DH_FIFO_SRC_REG_WTM_MASK                (1 << 7)
#define LIS3DH_FIFO_SRC_REG_OVRN_FIFO_MASK          (1 << 6)
#define LIS3DH_FIFO_SRC_REG_EMPTY_MASK              (1 << 5)
#define LIS3DH_FIFO_SRC_REG_FSS4_MASK               (1 << 4)
#define LIS3DH_FIFO_SRC_REG_FSS3_MASK               (1 << 3)
#define LIS3DH_FIFO_SRC_REG_FSS2_MASK               (1 << 2)
#define LIS3DH_FIFO_SRC_REG_FSS1_MASK               (1 << 1)
#define LIS3DH_FIFO_SRC_REG_FSS0_MASK               (1 << 0)
#define LIS3DH_FIFO_SRC_REG_FSS_SHIFT               (0)
#define LIS3DH_FIFO_SRC_REG_FSS_MASK                (LIS3DH_FIFO_SRC_REG_FSS0_MASK | \
                                                     LIS3DH_FIFO_SRC_REG_FSS1_MASK | \
                                                     LIS3DH_FIFO_SRC_REG_FSS2_MASK | \
                                                     LIS3DH_FIFO_SRC_REG_FSS3_MASK | \
                                                     LIS3DH_FIFO_SRC_REG_FSS4_MASK)
/** @} */ /* FIFO_CTRL_REG bitfield macros */

/**
 * @name    Register address bitfield macros
 * @{
 */
/**
 * @brief   Write to register
 */
#define LIS3DH_SPI_WRITE_MASK                       (0 << 7)
/**
 * @brief   The READ bit must be set when reading
 */
#define LIS3DH_SPI_READ_MASK                        (1 << 7)
/**
 * @brief   Multi byte transfers must assert this bit when writing the address.
 */
#define LIS3DH_SPI_MULTI_MASK                       (1 << 6)
/**
 * @brief   Opposite of LIS3DH_SPI_MULTI_MASK.
 */
#define LIS3DH_SPI_SINGLE_MASK                      (0 << 6)
/**
 * @brief   Mask of the address bits in the address byte during transfers.
 */
#define LIS3DH_SPI_ADDRESS_MASK                     (0x3F)
/** @} */ /* Register address bitfield macros */

/**
 * @brief   Length of scalar measurement data in bytes.
 */
#define LIS3DH_ADC_DATA_SIZE                        (2U)

/**
 * @name    FIFO modes.
 *
 * Used when calling lis3dh_set_fifo()
 * @{
 */
/**
 * @brief   FIFO mode: Bypass
 */
#define LIS3DH_FIFO_MODE_BYPASS                     (0x00 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: FIFO
 */
#define LIS3DH_FIFO_MODE_FIFO                       (0x01 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: Stream
 */
#define LIS3DH_FIFO_MODE_STREAM                     (0x02 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/**
 * @brief   FIFO mode: Stream to FIFO
 */
#define LIS3DH_FIFO_MODE_STREAM_TO_FIFO             (0x03 << LIS3DH_FIFO_CTRL_REG_FM_SHIFT)
/** @} */

// /**
//  * @name    Output Data Rates (ODR) macros
//  *
//  * Use these when calling lis3dh_set_odr(odr).
//  * @{
//  */
// /**
//  * @brief    Powerdown mode
//  */
// #define LIS3DH_ODR_POWERDOWN                        (0x00 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   1Hz mode
//  */
// #define LIS3DH_ODR_1Hz                              (0x01 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   10Hz mode
//  */
// #define LIS3DH_ODR_10Hz                             (0x02 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   25Hz mode
//  */
// #define LIS3DH_ODR_25Hz                             (0x03 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   50Hz mode
//  */
// #define LIS3DH_ODR_50Hz                             (0x04 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   100Hz mode
//  */
// #define LIS3DH_ODR_100Hz                            (0x05 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   200Hz mode
//  */
// #define LIS3DH_ODR_200Hz                            (0x06 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   400Hz mode
//  */
// #define LIS3DH_ODR_400Hz                            (0x07 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   Low power 1600Hz mode
//  */
// #define LIS3DH_ODR_LP1600Hz                         (0x08 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   Normal mode 1250 Hz
//  * @note    Normal mode 1250 Hz and Low power mode 5000 Hz share the same setting
//  */
// #define LIS3DH_ODR_NP1250Hz                         (0x09 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /**
//  * @brief   Low power mode 5000 Hz
//  * @note    Normal mode 1250 Hz and Low power mode 5000 Hz share the same setting
//  */
// #define LIS3DH_ODR_LP5000HZ                         (0x09 << LIS3DH_CTRL_REG1_ODR_SHIFT)
// /** @} */



#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_INTERNAL_H */
/** @} */