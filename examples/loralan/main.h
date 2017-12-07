/*
 * Copyright (C) 2016 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file
 * @brief       
 * @author      Evgeniy Ponomarev
 */
#ifndef LORALAN_DEVICE_MAIN_H_
#define LORALAN_DEVICE_MAIN_H_

#define DISPLAY_JOINKEY_2BYTES 1
#define DISPLAY_DEVNONCE_BYTE 1

#include "shell.h"
#include "unwds-common.h"

typedef enum {
    UNWDS_BOOT_NORMAL_MODE = 0,
    UNWDS_BOOT_SAFE_MODE = 1,
    UNWDS_BOOT_MODULES_FAILED = 2,
} boot_modes_t;

extern void init_node(shell_command_t *commands);

#endif /* LORALAN_DEVICE_MAIN_H_ */
