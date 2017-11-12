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
#ifndef LORA_STAR_UNI_MAIN_H_
#define LORA_STAR_UNI_MAIN_H_

#include "shell.h"

#define FIRMWARE_VERSION "1.61"

typedef int (*cmd_fun_t)(int, char **);

extern void init_node(shell_command_t **commands);
extern void init_gate(shell_command_t **commands);
extern void init_no_eui64(shell_command_t **commands);
extern void init_no_cfg(shell_command_t **commands);

void blink_led(void);

#endif /* LORA_STAR_UNI_MAIN_H_ */
