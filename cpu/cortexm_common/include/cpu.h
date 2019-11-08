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

extern uint32_t __attribute__((section(".noinit"))) cpu_jump_to_bootloader;

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
 * @brief   Pattern to write into the co-processor Access Control Register to
 *          allow full FPU access
 *
 * Used in the @ref cortexm_init_fpu inline function below.
 */
#define CORTEXM_SCB_CPACR_FPU_ACCESS_FULL         (0x00f00000)

/**
 * @brief   Initialization of the CPU
 */
void cpu_init(void);

/**
 * @brief   Initialize Cortex-M specific core parts of the CPU
 *
 * @ref cortexm_init calls, in a default order, @ref cortexm_init_fpu,
 * @ref cortexm_init_isr_priorities, and @ref cortexm_init_misc.  Also
 * performs other default initialisations, including ones which you
 * may or may not want.
 *
 * Unless you have special requirements (like nRF52 with SD has), it
 * is sufficient to call just @ref cortexm_init and the `cortexm_init_*`
 * functions do not need to (and should not) be called separately.
 * If you have conflicting requirements, you may want to have a look
 * `cpu/nrft/cpu.c` for an example of a non-default approach.
 */
void cortexm_init(void);

/**
 * @brief   Initialize Cortex-M FPU
 *
 * Called from `cpu/nrf52/cpu.c`, since it cannot use the
 * whole @ref cortexm_init due to conflicting requirements.
 *
 * Defined here as a static inline function to allow all
 * callers to optimise this away if the FPU is not used.
 */
static inline void cortexm_init_fpu(void)
{
    /* initialize the FPU on Cortex-M4F CPUs */
#if defined(CPU_ARCH_CORTEX_M4F) || defined(CPU_ARCH_CORTEX_M7)
    /* give full access to the FPU */
    SCB->CPACR |= (uint32_t)CORTEXM_SCB_CPACR_FPU_ACCESS_FULL;
#endif
}

#if defined(CPU_CORTEXM_INIT_SUBFUNCTIONS) || defined(DOXYGEN)

/**
 * @brief   Initialize Cortex-M interrupt priorities
 *
 * Called from `cpu/nrf52/cpu.c`, since it cannot use the
 * whole @ref cortexm_init due to conflicting requirements.
 *
 * Define `CPU_CORTEXM_INIT_SUBFUNCTIONS` to make this function
 * publicly available.
 */
void cortexm_init_isr_priorities(void);

/**
 * @brief   Initialize Cortex-M misc functions
 *
 * Called from `cpu/nrf52/cpu.c`, since it cannot use the
 * whole @ref cortexm_init due to conflicting requirements.
 *
 * Define `CPU_CORTEXM_INIT_SUBFUNCTIONS` to make this function
 * publicly available.
 */
void cortexm_init_misc(void);

#endif /* defined(CPU_CORTEXM_INIT_SUBFUNCTIONS) || defined(DOXYGEN) */

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

/*
 * @brief   Jumps to another image in flash
 *
 * This function is supposed to be called by a bootloader application.
 *
 * @param[in]   image_address   address in flash of other image
 */
static inline void cpu_jump_to_image(uint32_t image_address)
{
    /* Disable IRQ */
    /*__disable_irq();*/ /* breaks STM32 DFU bootloader */

    /* set MSP */
    __set_MSP(*(uint32_t*)image_address);

    /* skip stack pointer */
    image_address += 4;

    /* load the images reset_vector address */
    uint32_t destination_address = *(uint32_t*)image_address;

    /* Make sure the Thumb State bit is set. */
    destination_address |= 0x1;

    /* Branch execution */
    __asm("BX %0" :: "r" (destination_address));
}

/* The following register is only present for Cortex-M0+, -M3, -M4 and -M7 CPUs */
#if defined(CPU_ARCH_CORTEX_M0PLUS) || defined(CPU_ARCH_CORTEX_M3) || \
    defined(CPU_ARCH_CORTEX_M4) || defined(CPU_ARCH_CORTEX_M4F) || \
    defined(CPU_ARCH_CORTEX_M7)
static inline uint32_t cpu_get_image_baseaddr(void)
{
    return SCB->VTOR;
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* CPU_H */
/** @} */
