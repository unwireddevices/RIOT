/*
 * Copyright (C) 2016-2018 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     
 * @{
 *
 * @file
 * @brief       
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */

#ifndef NDEF_H
#define NDEF_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct __attribute__((packed)) {
    uint8_t MB:1;
    uint8_t ME:1;
    uint8_t CF:1;
    uint8_t SR:1;
    uint8_t ID:1;
    uint8_t TNF:3;
} __attribute() msg_flag_t;

typedef struct __attribute__((packed)) {
    msg_flag_t msg_flag;
    uint8_t    type_len;
    uint32_t   payload_len;
    uint8_t    id_len;
    


}

#ifdef __cplusplus
}
#endif

#endif /*NDEF_H*/

