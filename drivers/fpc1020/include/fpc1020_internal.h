/*
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_fpc1020 FPC1020 capacitive fingerprint sensor
 * @ingroup     drivers_sensors
 * @brief       Device driver interface for the FPC1020 sensor.
 *
 * @author      Oleg Artamonov <info@unwds.com>
 */

#ifndef FPC1020_INTERNAL_H
#define FPC1020_INTERNAL_H

#include <inttypes.h>
#include "periph/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /* --- Common registers --- */
    FPC102X_REG_FPC_STATUS              = 20,    /* RO, 1 bytes  */
    FPC102X_REG_READ_IRQ                = 24,    /* RO, 1 byte   */
    FPC102X_REG_READ_IRQ_WITH_CLEAR     = 28,    /* RO, 1 byte   */
    FPC102X_REG_READ_ERROR_WITH_CLEAR   = 56,    /* RO, 1 byte   */
    FPC102X_REG_MISO_EDGE_RIS_EN        = 64,    /* WO, 1 byte   */
    FPC102X_REG_FPC_CONFIG              = 68,    /* RW, 1 byte   */
    FPC102X_REG_IMG_SMPL_SETUP          = 76,    /* RW, 3 bytes  */
    FPC102X_REG_CLOCK_CONFIG            = 80,    /* RW, 1 byte   */
    FPC102X_REG_IMG_CAPT_SIZE           = 84,    /* RW, 4 bytes  */
    FPC102X_REG_IMAGE_SETUP             = 92,    /* RW, 1 byte   */
    FPC102X_REG_ADC_TEST_CTRL           = 96,    /* RW, 1 byte   */
    FPC102X_REG_IMG_RD                  = 100,    /* RW, 1 byte   */
    FPC102X_REG_SAMPLE_PX_DLY           = 104,    /* RW, 8 bytes  */
    FPC102X_REG_PXL_RST_DLY             = 108,    /* RW, 1 byte   */
    FPC102X_REG_TST_COL_PATTERN_EN      = 120,    /* RW, 2 bytes  */
    FPC102X_REG_CLK_BIST_RESULT         = 124,    /* RW, 4 bytes  */
    FPC102X_REG_ADC_WEIGHT_SETUP        = 132,    /* RW, 1 byte   */
    FPC102X_REG_ANA_TEST_MUX            = 136,    /* RW, 4 bytes  */
    FPC102X_REG_FINGER_DRIVE_CONF       = 140,    /* RW, 1 byte   */
    FPC102X_REG_FINGER_DRIVE_DLY        = 144,    /* RW, 1 byte   */
    FPC102X_REG_OSC_TRIM                = 148,    /* RW, 2 bytes  */
    FPC102X_REG_ADC_WEIGHT_TABLE        = 152,    /* RW, 10 bytes */
    FPC102X_REG_ADC_SETUP               = 156,    /* RW, 5 bytes  */
    FPC102X_REG_ADC_SHIFT_GAIN          = 160,    /* RW, 2 bytes  */
    FPC102X_REG_BIAS_TRIM               = 164,    /* RW, 1 byte   */
    FPC102X_REG_PXL_CTRL                = 168,    /* RW, 2 bytes  */
    FPC102X_REG_CAPTURE_IMAGE           = 192,    /* RO, 0 bytes  */
    FPC102X_REG_READ_IMAGE              = 196,    /* RO, variable size  */
    FPC102X_REG_FPC_DEBUG               = 208,    /* RO, 1 bytes  */
    FPC102X_REG_FINGER_PRESENT_STATUS   = 212,    /* RO, 2 bytes  */
    FPC102X_REG_HWID                    = 252,    /* RO, 2 bytes  */
    /* --- fpc1020/21 specific --- */
    FPC1020_REG_FNGR_DET_THRES          = 216,    /* RW, 1 byte   */
    FPC1020_REG_FNGR_DET_CNTR           = 220,    /* RW, 2 bytes  */
    /* --- fpc1150 specific --- */
    FPC1150_REG_OFFSET                  = 1000,    /* Not a register ! */
    FPC1150_REG_FNGR_DET_THRES          = 1216,    /* RW, 4 byte   */
    FPC1150_REG_FNGR_DET_CNTR           = 1220,    /* RW, 4 bytes  */
} fpc1020_reg_t;

typedef enum {
    FPC_1020_IRQ_REG_BIT_FINGER_DOWN = 1 << 0,
    FPC_1020_IRQ_REG_BIT_ERROR = 1 << 2,
    FPC_1020_IRQ_REG_BIT_FIFO_NEW_DATA = 1 << 5,
    FPC_1020_IRQ_REG_BIT_COMMAND_DONE = 1 << 7,
    FPC_1020_IRQ_REG_BITS_REBOOT = 0xff
} fpc1020_irq_reg_t;

typedef enum {
	FPC1020_MODE_IDLE = 0,
	FPC1020_MODE_WAIT_AND_CAPTURE = 1,
	FPC1020_MODE_SINGLE_CAPTURE = 2,
	FPC1020_MODE_CHECKERBOARD_TEST_NORM = 3,
	FPC1020_MODE_CHECKERBOARD_TEST_INV = 4,
	FPC1020_MODE_BOARD_TEST_ONE = 5,
	FPC1020_MODE_BOARD_TEST_ZERO = 6,
	FPC1020_MODE_WAIT_FINGER_DOWN = 7,
	FPC1020_MODE_WAIT_FINGER_UP = 8,
	FPC1020_MODE_WAIT_FINGER_UP_AND_CAPTURE = 9,
} fpc1020_capture_mode_t;

#ifdef __cplusplus
}
#endif

#endif /* FPC1020_INTERNAL_H */
/** @} */
