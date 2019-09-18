/*
 * Copyright (C) 2018 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */


/**
 * @defgroup    
 * @ingroup     sys
 * @brief       Shell password protection
 * @{
 *
 * @file
 * @brief       Shell password protection
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 */

#ifndef SHELL_PASSWORD_H
#define SHELL_PASSWORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdbool.h>

const char *SHELL_DEFAULT_PASSWORD = "12345";
#define SHELL_PASSWORD_MAX_LENGTH   20

/**
 * @brief           Saves shell password (if password protection is enabled)
 *
 * @param[in]       password    string with the new password
 */
void __attribute__((weak)) shell_set_password(char* password) {
    (void)password;
    printf("Password change not supported, default is %s\n", SHELL_DEFAULT_PASSWORD);
}

/**
 * @brief           Gets stored shell password (if password protection is enabled)
 *
 * @param[in]       password    string to copy stored password to
 */
void __attribute__((weak)) shell_get_password(char* password) {
    strcpy(password, SHELL_DEFAULT_PASSWORD);
}

#ifdef __cplusplus
}
#endif

#endif /* SHELL_PASSWORD_H */
/** @} */
