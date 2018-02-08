/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Shell password protection
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include <string.h>

#include "shell_password.h"

bool shell_password_enabled;
bool shell_password_entered;
char shell_password[10];

int _password_handler(int argc, char **argv)
{
    if (shell_password_enabled) {
        if (shell_password_entered && (strcmp(argv[1], "set") == 0)) {
            shell_set_password(argv[1]);
        } else {
            char password[10];
            shell_get_password(password);
            
            if (strcmp(argv[1], password) == 0) {
                shell_password_entered = true;
                puts("Shell unlocked");
            } else {
                puts("Shell incorrect password");
            }
        }
    } else {
        puts("Shell: password protection is not enabled");
    }

    return 0;
}
