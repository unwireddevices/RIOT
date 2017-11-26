/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Default application that shows a functionality of LoRa-Star gateway
 *
 * @author      Eugene Ponomarev
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ls-init-device.h"
#include "main.h"

static shell_command_t shell_commands_node[UNWDS_SHELL_COMMANDS_MAX] = {};

int main(void)
{
    init_role(shell_commands_node);

    return 0;
}
