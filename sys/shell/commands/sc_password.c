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
#include <assert.h>

#include "shell_password.h"

bool shell_password_entered;
char shell_password[10];

int _password_handler(int argc, char **argv)
{   
    if (argc < 2) {
        puts("password <pwd> - unlocks shell");
        puts("password get - prints current password");
        puts("password set - sets new password");
        return 0;
    }

    char password[SHELL_PASSWORD_MAX_LENGTH];
    shell_get_password(password);
    
    if (shell_password_entered && (strcmp(argv[1], "set") == 0)) {
        shell_set_password(argv[1]);
        return 0;
    }
    
    if (shell_password_entered && (strcmp(argv[1], "get") == 0)) {
        shell_get_password(password);
        printf("Shell password: %s\n", password);
        return 0;
    }

    if (strcmp(argv[1], password) == 0) {
        shell_password_entered = true;
        puts("Shell unlocked");
    } else {
        puts("Incorrect shell password");
    }

    return 0;
}
