/*
 * Copyright (C) 2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_nrf5x_nrfmax_gnrc GNRC adapter for nrfmax
 * @ingroup     drivers_nrf5x_nrfmax
 * @brief       Minimal driver for the NRF51 radio
 *
 * @{
 *
 * @file
 * @brief       GNRC adapter for nrfmax devices (e.g. nRF5x radios)
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 */

#ifndef NRFMAX_GNRC_H
#define NRFMAX_GNRC_H

#include "nrfmax.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Initialize the nrfmax GNRC adapter, also takes care of the nrfmax
 *          driver setup
 *
 * As we have never more than 1 nrfmax device on a board, we can make some
 * simplifications when it come to allocating device descriptors and adapter
 * data structures -> we do this right in the driver/adapter code, so this
 * function can be called from auto_init as is, without the need for external
 * memory allocation.
 */
void gnrc_netdev_nrfmax_init(void);

#ifdef __cplusplus
}
#endif

#endif /* NRFMAX_GNRC_H */
/** @} */
