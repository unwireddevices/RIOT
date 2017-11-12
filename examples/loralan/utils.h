/*
 * Copyright (C) 2016 cr0s
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
 * @author      cr0s
 */
#ifndef LORA_STAR_UNI_UTILS_H_
#define LORA_STAR_UNI_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

bool hex_to_bytes(char *hexstr, uint8_t *bytes, bool reverse_order);
bool hex_to_bytesn(char *hexstr, uint32_t len, uint8_t *bytes, bool reverse_order);
void bytes_to_hex(uint8_t *bytes, size_t num_bytes, char *str, bool reverse_order);
bool is_number(char* str);

#endif /* LORA_STAR_UNI_UTILS_H_ */
