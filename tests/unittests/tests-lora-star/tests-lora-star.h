/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @addtogroup  unittests
 * @brief       
 * @{
 * @file		tests-lora-star.h
 * @brief       LoRa-Star network MAC level unit tests definitions
 * @author      Eugene Ponomarev
 */
#ifndef TESTS_UNITTESTS_TESTS_LORA_STAR_TESTS_LORA_STAR_H_
#define TESTS_UNITTESTS_TESTS_LORA_STAR_TESTS_LORA_STAR_H_

#include "embUnit/embUnit.h"

/**
 * @brief   Generates tests for ls-crypto.h
 *
 * @return  embUnit tests if successful, NULL if not.
 */
Test *tests_lora_star_ls_crypto_tests(void);

/**
 * @brief   Generates tests for ls-frame-fifo.h
 *
 * @return  embUnit tests if successful, NULL if not.
 */
Test *tests_lora_star_ls_frame_fifo_tests(void);

/**
 * @brief   Generates tests for ls-mac.h
 *
 * @return  embUnit tests if successful, NULL if not.
 */
Test *tests_lora_star_ls_mac_tests(void);

#endif /* TESTS_UNITTESTS_TESTS_LORA_STAR_TESTS_LORA_STAR_H_ */
