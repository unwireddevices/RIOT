/*
 * Copyright (C) 2013 INRIA
 *               2014 Freie Universität Berlin
 *               2016 TriaGnoSys GmbH
 *               2018 Kaspar Schleiser <kaspar@schleiser.de>
 *               2018 OTA keys S.A.
 *
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @{
 *
 * @file
 * @brief       Implementation of the kernel cpu functions
 *
 * @author      Stefan Pfeiffer <stefan.pfeiffer@fu-berlin.de>
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Nick van IJzendoorn <nijzendoorn@engineering-spirit.nl>
 * @author      Víctor Ariño <victor.arino@zii.aero>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include "cpu.h"
#include "stmclk.h"
#include "periph_cpu.h"
#include "periph/init.h"
#include "periph/rtc.h"
#include "periph/gpio.h"
#include "board.h"

#if defined (CPU_FAM_STM32L4)
#define BIT_APB_PWREN       RCC_APB1ENR1_PWREN
#else
#define BIT_APB_PWREN       RCC_APB1ENR_PWREN
#endif

static void jump_to_bootloader(void) __attribute__ ((noreturn));

/* Sets up and jumps to the bootloader */
static void jump_to_bootloader(void) {
    /* System memory is the valid area next _below_ Option bytes */
    char *a, *b, *c;
    a = (char *)(OB_BASE - 1);
    b = 0;
    
    /* Here we have System memory top address */
    c = cpu_find_next_valid_address(a, b, true);
    
    /* Here we have System memory bottom address */
    c = cpu_find_next_valid_address(c, b, false) + 1;
    
    if (!c) {
        NVIC_SystemReset();
    }
    
    uint32_t boot_addr = (uint32_t)c;
    
    uint32_t boot_stack_ptr = *(uint32_t*)(boot_addr);
    uint32_t dfu_reset_addr = *(uint32_t*)(boot_addr+4);

    void (*dfu_bootloader)(void) = (void (*))(dfu_reset_addr);

    /* Remap vector table to system memory */
    RCC->APB2ENR |= 1;
    #if defined(CPU_FAM_STM32F0)
        SYSCFG->CFGR1 = 0x1;
    #else
        SYSCFG->MEMRMP = 0x1;
    #endif

    /* Reset the stack pointer */
    __set_MSP(boot_stack_ptr);

    dfu_bootloader();
    while (1);
}

void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();
       
    /* enable PWR module */
    periph_clk_en(APB1, BIT_APB_PWREN);

    /* check if we need to update firmware */
    rtc_poweron();
    if (rtc_restore_backup(RTC_REGBACKUP_BOOTLOADER) == RTC_REGBACKUP_BOOTLOADER_VALUE) {
        /* clear RTC register */
        rtc_save_backup(0, RTC_REGBACKUP_BOOTLOADER);
        rtc_poweroff();
        
        jump_to_bootloader();
    }
    rtc_poweroff();
    
    /* initialize the system clock as configured in the periph_conf.h */
    stmclk_init_sysclk();
    
#if defined(CPU_FAM_STM32L1)
    /* switch all GPIOs to AIN mode to minimize power consumption */
    uint8_t i;
    GPIO_TypeDef *port;
    
    /* enable GPIO clock */
    uint32_t ahb_gpio_clocks = RCC->AHBENR & 0xFF;
    periph_clk_en(AHB, 0xFF);
    
    for (i = 0; i < 8; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        if (cpu_check_address((char *)port)) {
            port->MODER = 0xffffffff;
        } else {
            break;
        }
    }
    
    /* restore GPIO clock */
    uint32_t tmpreg = RCC->AHBENR;
    tmpreg &= ~((uint32_t)0xFF);
    tmpreg |= ahb_gpio_clocks;
    periph_clk_en(AHB, tmpreg);
#endif

#ifdef MODULE_PERIPH_DMA
    /*  initialize DMA streams */
    dma_init();
#endif
    /* trigger static peripheral initialization */
    periph_init();
    
    cpu_init_status();
}
