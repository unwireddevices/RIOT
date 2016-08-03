/*
 * Copyright (C) 2016 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#include <stdlib.h>
#include <stdint.h>

#include "embUnit/embUnit.h"
#include "ls_crypto.h"
#include "tests-lora-star.h"

static void test_ls_calculate_mic_1(void) {
    /* ... */

    TEST_ASSERT(NULL);
}

Test *tests_lora_star_ls_crypto_tests(void)
{
    EMB_UNIT_TESTFIXTURES(fixtures) {
        new_TestFixture(test_ls_calculate_mic_1),
    };

    EMB_UNIT_TESTCALLER(lora_star_ls_crypto_tests, "lora_star_ls_crypto_tests",
    		NULL,
			NULL, fixtures);

    /* set up and tear down function can be NULL if omitted */

    return (Test *)&lora_star_ls_crypto;
}
