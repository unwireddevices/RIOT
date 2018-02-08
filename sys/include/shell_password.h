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

/**
 * @brief           Saves shell password (if password protection is enabled)
 *
 * @param[in]       password    string with the new password
 */
void __attribute__((weak)) shell_set_password(char* password) {
    puts("Password change not supported, default is 12345");
}

/**
 * @brief           Gets stored shell password (if password protection is enabled)
 *
 * @param[in]       password    string to copy stored password to
 */
void __attribute__((weak)) shell_get_password(char* password) {
    snprintf(password, 10, "12345");
}

#ifdef __cplusplus
}
#endif

#endif /* SHELL_PASSWORD_H */
/** @} */
