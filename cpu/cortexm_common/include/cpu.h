/*
 * Copyright (C) 2014-2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    cpu_cortexm_common ARM Cortex-M common
 * @ingroup     cpu
 * @brief       Common implementations and headers for Cortex-M family based
 *              micro-controllers
 * @{
 *
 * @file
 * @brief       Basic definitions for the Cortex-M common module
 *
 * When ever you want to do something hardware related, that is accessing MCUs
 * registers, just include this file. It will then make sure that the MCU
 * specific headers are included.
 *
 * @author      Stefan Pfeiffer <stefan.pfeiffer@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @todo        remove include irq.h once core was adjusted
 */

#ifndef CPU_H
#define CPU_H

#include <stdio.h>

#include "irq.h"
#include "sched.h"
#include "thread.h"
#include "cpu_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Interrupt stack canary value
 *
 * @note 0xe7fe is the ARM Thumb machine code equivalent of asm("bl #-2\n") or
 * 'while (1);', i.e. an infinite loop.
 * @internal
 */
#define STACK_CANARY_WORD   (0xE7FEE7FEu)

/**
 * @brief   All Cortex-m-based CPUs provide pm_set_lowest
 *
 * The pm_set_lowest is provided either by the pm_layered module if used, or
 * alternatively as fallback by the cortexm's own implementation.
 */
#define PROVIDES_PM_SET_LOWEST

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    #define CPU_NAME_MAX_SIZE           20
    #define CPU_CLOCK_SOURCE_MAX_SIZE   9
#endif

typedef struct {
    size_t      size;
    uint32_t    pages;
    uint32_t    pagesize;
    uint8_t     alignment;
} cpu_flash_t;

typedef struct {
    size_t      size;
    uint8_t     alignment;
} cpu_ram_t;

typedef struct {
    size_t      size;
    uint8_t     alignment;
} cpu_eeprom_t;

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
typedef struct {
    int16_t         vdd;        /**< CPU VDD voltage, mV (INT16_MIN if not available)  */
    int16_t         vdda;       /**< CPU VDDA voltage, mV (INT16_MIN if not available) */
    int16_t         vbat;       /**< CPU VBAT voltage, mV (INT16_MIN if not available) */
} cpu_voltage_t;

typedef struct {
    int16_t         core_temp;          /**< CPU core temperature, C (INT16_MIN if not available) */
} cpu_temp_t;
#endif

typedef struct {
    uint32_t        coreclock;          /**< CPU core clock     */
#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    char            source[CPU_CLOCK_SOURCE_MAX_SIZE + 1];         /**< CPU clock source   */
#endif
} cpu_clock_t;

typedef struct {
    cpu_ram_t       ram;                /**< CPU RAM data */
    cpu_flash_t     flash;              /**< CPU flash data */
    cpu_eeprom_t    eeprom;             /**< CPU EEPROM data */
    cpu_clock_t     clock;              /**< CPU clocks */
#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    cpu_voltage_t   voltage;            /**< CPU voltages */
    cpu_temp_t      temp;               /**< CPU core temperatures */
    char            model[CPU_NAME_MAX_SIZE];    /**< CPU name and model */
#endif
    uint8_t         category;           /**< CPU category (0 if not applicable) */
} cpu_status_t;

extern cpu_status_t cpu_status;

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
/**
 * @brief   Updates CPU status info
 */
void cpu_update_status(void);
#endif

/**
 * @brief   Initializes CPU status info
 */
void cpu_init_status(void);

/**
 * @brief   Initialization of the CPU
 */
void cpu_init(void);

/**
 * @brief   Initialize Cortex-M specific core parts of the CPU
 */
void cortexm_init(void);

/**
 * @brief   Prints the current content of the link register (lr)
 */
static inline void cpu_print_last_instruction(void)
{
    uint32_t *lr_ptr;
    __asm__ __volatile__("mov %0, lr" : "=r"(lr_ptr));
    printf("%p\n", (void*) lr_ptr);
}

/**
 * @brief   Put the CPU into the 'wait for event' sleep mode
 *
 * This function is meant to be used for short periods of time, where it is not
 * feasible to switch to the idle thread and back.
 */
static inline void cortexm_sleep_until_event(void)
{
    __WFE();
}

/**
 * @brief   Put the CPU into (deep) sleep mode, using the `WFI` instruction
 *
 * @param[in] deep      !=0 for deep sleep, 0 for light sleep
 */
static inline void cortexm_sleep(int deep)
{
    if (deep) {
        SCB->SCR |=  (SCB_SCR_SLEEPDEEP_Msk);
    }
    else {
        SCB->SCR &= ~(SCB_SCR_SLEEPDEEP_Msk);
    }

    unsigned state = irq_disable();
    /* ensure that all memory accesses have completed and trigger sleeping */
#if defined (__CC_ARM)
    __force_stores();
#endif

    __DSB();
    __WFI();
    
    irq_restore(state);
}

/**
 * @brief   Trigger a conditional context scheduler run / context switch
 *
 * This function is supposed to be called in the end of each ISR.
 */
static inline void cortexm_isr_end(void) {
    if (sched_context_switch_request) {
        thread_yield_higher();
    }
}

/**
 * @brief   Checks is memory address valid or not
 *
 * This function can be used to check for memory size,
 * peripherals availability, etc.
 * 
 * @param[in]	address     Address to check
 */
bool cpu_check_address(volatile const char *address);

/**
 * @brief   Checks for next valid or next invalid address
 *
 * This function can be used to find specific memory areas
 * 
 * @param[in]	start    Start at address
 * @param[in]	stop     Stop at address
 * @param[in]	step     Address step
 * @param[in]	valid    Address type to look for
 */
char* cpu_find_next_valid_address(char *start, char *stop, uint32_t step, bool valid);

/**
 * @brief   Determine CPU memory size
 *
 * This function can be used to calculate memory size in runtime
 * 
 * @param[in]	base     Address to start
 * @param[in]   block    Default increment
 * @param[in]   maxsize  Maximum possible memory size
 */
size_t cpu_find_memory_size(char *base, uint32_t block, uint32_t maxsize);

#ifdef __cplusplus
}
#endif

#endif /* CPU_H */
/** @} */
