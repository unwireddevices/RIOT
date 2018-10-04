/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_unwd-range
 * @ingroup     boards
 * @brief       Board specific files for the Unwired Range boards
 * @{
 *
 * @file
 * @brief       Board specific definitions for the Unwired Range boards
 *
 * @author      Mikhail Churikov
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#ifndef UNWD_COMMON_BOARD_H_
#define UNWD_COMMON_BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UNWD_COMMON_BOARD_H_ */
/** @} */
