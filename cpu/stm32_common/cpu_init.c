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
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @author      Francisco Molina <francisco.molina@inria.cl>
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

#if defined(CPU_JUMP_TO_BOOTLOADER_MAGIC_KEY)
uint32_t __attribute__((section(".noinit"))) cpu_jump_to_bootloader;

static void jump_to_bootloader(void) __attribute__ ((noreturn));

/* Sets up and jumps to the bootloader */
static void jump_to_bootloader(void) {
    /* System memory is the valid area next _below_ Option bytes */
    char *a, *b, *c;
    a = (char *)(OB_BASE - 4);
    b = 0;

    /* Here we have System memory top address */
    c = cpu_find_next_valid_address(a, b, 4, true);

    /* Here we have System memory bottom address */
    c = cpu_find_next_valid_address(c, b, 4, false) + 4;

    if (!c) {
        NVIC_SystemReset();
    }

#if defined(LED0_PIN)
    gpio_init(LED0_PIN, GPIO_OUT);
    gpio_set(LED0_PIN);
#endif

    uint32_t boot_addr = (uint32_t)c;
    cpu_jump_to_image(boot_addr);

    while (1);
}
#endif

#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F3) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32F7) || \
    defined(CPU_FAM_STM32L1)

#define STM32_CPU_MAX_GPIOS    (12U)

#if defined(CPU_FAM_STM32L1)
#define GPIO_CLK              (AHB)
#define GPIO_CLK_ENR          (RCC->AHBENR)
#define GPIO_CLK_ENR_MASK     (0x0000FFFF)
#elif defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F3)
#define GPIO_CLK              (AHB)
#define GPIO_CLK_ENR          (RCC->AHBENR)
#define GPIO_CLK_ENR_MASK     (0xFFFF0000)
#elif defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F4) || \
      defined(CPU_FAM_STM32F7)
#define GPIO_CLK              (AHB1)
#define GPIO_CLK_ENR          (RCC->AHB1ENR)
#define GPIO_CLK_ENR_MASK     (0x0000FFFF)
#elif defined(CPU_FAM_STM32F1)
#define GPIO_CLK              (APB2)
#define GPIO_CLK_ENR          (RCC->APB2ENR)
#define GPIO_CLK_ENR_MASK     (0x000001FC)
#endif

#ifndef DISABLE_JTAG
#define DISABLE_JTAG 1
#endif

#ifndef DISABLE_SWD
#define DISABLE_SWD 0
#endif

/**
 * @brief   Initialize gpio to AIN
 *
 * stm32f need to have all there pins initialized to AIN so the consumption
 * of the input Schmitt trigger is saved when running in STOP mode.
 *
 * @see https://comm.eefocus.com/media/download/index/id-1013834
 */
static void _gpio_init_ain(void)
{
    uint32_t ahb_gpio_clocks;

    /* enable GPIO clock and save GPIO clock configuration */
    ahb_gpio_clocks = GPIO_CLK_ENR & GPIO_CLK_ENR_MASK;
    periph_clk_en(GPIO_CLK, GPIO_CLK_ENR_MASK);

    /* switch all GPIOs to AIN mode to minimize power consumption */
    for (uint8_t i = 0; i < STM32_CPU_MAX_GPIOS; i++) {
        GPIO_TypeDef *port;
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        if (IS_GPIO_ALL_INSTANCE(port) && cpu_check_address((char *)port)) {
            if ((!DISABLE_JTAG) || (!DISABLE_SWD)) {
#if defined(CPU_FAM_STM32F1)
                switch (i) {
                    /* preserve JTAG pins on PORTA and PORTB */
                    case 0:
                        port->CR[0] = GPIO_CRL_CNF;
                        port->CR[1] = GPIO_CRH_CNF & 0x000FFFFF;
                        break;
                    case 1:
                        if (DISABLE_JTAG) {
                            /* preserve SWD pins only */
                            port->CR[0] = GPIO_CRL_CNF & 0xFFF00FFF;
                            port->CR[1] = GPIO_CRH_CNF;
                        }
                        break;
                    default:
                        port->CR[0] = GPIO_CRL_CNF;
                        port->CR[1] = GPIO_CRH_CNF;
                        break;
                }
#else /* ! defined(CPU_FAM_STM32F1) */
                switch (i) {
                    /* preserve JTAG pins on PORTA and PORTB */
                    case 0:
                        if (DISABLE_JTAG) {
                            /* preserve SWD pins (PA13, PA14) only */
                            port->MODER = 0xEBFFFFFF;
                        }
                        else {
                            port->MODER = 0xABFFFFFF;
                        }
                        break;
                    case 1:
                        if (DISABLE_JTAG) {
                            /* preserve SWD pins (PA13, PA14) only */
                            port->MODER = 0xFFFFFFFF;
                        } else {
                            port->MODER = 0xFFFFFEBF;
                        }
                        break;
                    default:
                        port->MODER = 0xFFFFFFFF;
                        break;
                }
#endif /* defined(CPU_FAM_STM32F1) */
            }
            else {
#if defined(CPU_FAM_STM32F1)
                port->CR[0] = GPIO_CRL_CNF;
                port->CR[1] = GPIO_CRH_CNF;
#else
                port->MODER = 0xFFFFFFFF;
#endif
            }
        }
    }

    /* restore GPIO clocks */
    periph_clk_en(GPIO_CLK, ahb_gpio_clocks);
}
#endif

void cpu_init(void)
{
    /* initialize the Cortex-M core */
    cortexm_init();

    /* enable PWR module */
    periph_clk_en(APB1, BIT_APB_PWREN);

    /* check if we need to update firmware */
#if defined(CPU_JUMP_TO_BOOTLOADER_MAGIC_KEY)
    if (cpu_jump_to_bootloader == CPU_JUMP_TO_BOOTLOADER_MAGIC_KEY) {
        cpu_jump_to_bootloader = 0;
        jump_to_bootloader();
    }
#endif

    /* initialize the system clock as configured in the periph_conf.h */
    stmclk_init_sysclk();

#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F1) || \
    defined(CPU_FAM_STM32F2) || defined(CPU_FAM_STM32F3) || \
    defined(CPU_FAM_STM32F4) || defined(CPU_FAM_STM32F7) || \
    defined(CPU_FAM_STM32L1)
    _gpio_init_ain();
#endif

#ifdef MODULE_PERIPH_DMA
    /*  initialize DMA streams */
    dma_init();
#endif
    /* trigger static peripheral initialization */
    periph_init();

    cpu_init_status();
}
