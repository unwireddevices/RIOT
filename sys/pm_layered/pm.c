/*
 * Copyright (C) 2016 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_pm_layered
 * @{
 *
 * @file
 * @brief       Platform-independent power management code
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 *
 * @}
 */

#include "irq.h"
#include "periph/pm.h"
#include "pm_layered.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef PM_NUM_MODES
#error PM_NUM_MODES must be defined in periph_cpu.h!
#endif

#define PM_BLOCKER_INITIAL { .val_u32 = 0x01010101 }

/**
 * @brief Power Management mode typedef
 */
typedef union {
    uint32_t val_u32;
    uint8_t val_u8[PM_NUM_MODES];
} pm_blocker_t;

#if (PM_NUM_MODES > 4)
    #error PM_NUM_MODES must be <= 4!
#endif

/**
 * @brief Global variable for keeping track of blocked modes
 */
volatile pm_blocker_t pm_blocker = PM_BLOCKER_INITIAL;

void pm_set_lowest(void)
{
    pm_blocker_t blocker = pm_blocker;
    unsigned mode = PM_NUM_MODES;
    while (mode) {
        if (blocker.val_u8[mode-1]) {
            break;
        }
        mode--;
    }

    /* set lowest mode if blocker is still the same */
    unsigned state = irq_disable();
    if (blocker.val_u32 == pm_blocker.val_u32) {
        pm_set(mode);
    }
    
    irq_restore(state);    
}

void _pm_block(unsigned mode, const char* caller)
{
    unsigned state = irq_disable();
    if (pm_blocker.val_u8[mode] < 255) {
        pm_blocker.val_u8[mode]++;
    }
    irq_restore(state);
    DEBUG("pm: blocking mode %u by %s (%d)\n", mode, caller, pm_blocker.val_u8[mode]);
}

void _pm_unblock(unsigned mode, const char *caller)
{
    unsigned state = irq_disable();
    if (pm_blocker.val_u8[mode] > 0) {
        pm_blocker.val_u8[mode]--;
    }
    irq_restore(state);
    DEBUG("pm: unblocking mode %u by %s (%d)\n", mode, caller, pm_blocker.val_u8[mode]);
}

#ifndef PROVIDES_PM_LAYERED_OFF
void pm_off(void)
{
    pm_blocker.val_u32 = 0;
    pm_set_lowest();
    while(1) {}
}
#endif
