/*
 * Copyright (C) 2016 Unwired Devices
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
 * @brief       LoRa-Star frames assembly/disassembly definitions
 * @author      Eugene Ponomarev
 */
#ifndef UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_MAC_H_
#define UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_MAC_H_

#include "ls-mac-types.h"
#include "ls-crypto.h"

void ls_assemble_frame(ls_addr_t addr, ls_type_t type, uint8_t *buf, size_t buflen, ls_frame_t *frame);

bool ls_validate_frame(uint8_t *buf, size_t buflen);

#endif /* UNWIRED_MODULES_LORA_STAR_INCLUDE_LS_MAC_H_ */
