/*
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis3dh
 * @{
 *
 * @files       lis3dh_internal.h
 * @brief       Definition for the LIS3DH accelerometer
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */
#ifndef LIS3DH_INTERNAL_H
#define LIS3DH_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif


#define PROPERTY_DISABLE                            (0x00)
#define PROPERTY_ENABLE                             (0x01)

/**
 * @brief   Identifier register value
 *
 * The WHO_AM_I register should contain this value in order to correctly
 * identify the chip.
 */
#define LIS3DH_WHO_AM_I_RESPONSE                    (0x33)


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
    uint8_t i1_321da        : 1;
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


#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_INTERNAL_H */
/** @} */