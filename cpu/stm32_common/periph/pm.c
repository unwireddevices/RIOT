/*
 * Copyright (C) 2018 OTA keys S.A.
 *               2016 Kaspar Schleiser <kaspar@schleiser.de>
 *               2015 Freie Universität Berlin
 *               2015 Engineering-Spirit
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       Implementation of the kernels power management interface
 *
 * @author      Nick v. IJzendoorn <nijzndoorn@engineering-spirit.nl>
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */

#include "irq.h"
#include "stmclk.h"
#include "periph_cpu_common.h"

#include "periph/pm.h"
#include "periph/uart.h"
#include "periph/gpio.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef PM_STOP_CONFIG
/**
 * @brief Define config flags for stop mode
 *
 * Available values can be found in reference manual, PWR section, register CR.
 */
#if defined(CPU_FAM_STM32F0)
#define PM_STOP_CONFIG (PWR_CR_LPDS)
#else
#define PM_STOP_CONFIG (PWR_CR_LPDS | PWR_CR_FPDS)
#endif
#endif

static uint32_t lpm_gpio_moder[8];
static uint32_t lpm_gpio_pupdr[8];
static uint16_t lpm_gpio_otyper[8];
static uint8_t lpm_usart[UART_NUMOF];

#if defined (STM32L1XX_HD) || defined (STM32L1XX_XL)
static uint16_t lpm_gpio_brr[8];
#endif

static inline void _pm_before(void) {
	uint32_t i;
    GPIO_TypeDef *port;
	
	for (i = 0; i < 8; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));

        if (cpu_check_address((char *)port)) {
            /* save GPIO registers values */
            lpm_gpio_moder[i] = port->MODER;
            lpm_gpio_pupdr[i] = port->PUPDR;
            lpm_gpio_otyper[i] = (uint16_t)(port->OTYPER & 0xFFFF);
            #if defined (STM32L1XX_HD) || defined (STM32L1XX_XL)
                lpm_gpio_brr[i] = (uint16_t)(port->BRR & 0xFFFF);
            #endif
        } else {
            break;
        }
	}
	
    /* Disable all USART interfaces in use */
    /* without it, RX will receive some garbage when MODER is changed */
#if defined(UART_NUMOF)
    for (i = 0; i < UART_NUMOF; i++) {
        if (uart_config[i].dev->CR1 & USART_CR1_UE) {
            uart_config[i].dev->CR1 &= ~USART_CR1_UE;
            gpio_init(uart_config[i].tx_pin, GPIO_IN_PU);
            lpm_usart[i] = 1;
        } else {
            lpm_usart[i] = 0;
        }
    }
#endif

    /* specifically set GPIOs used for external SPI devices */
    /* MOSI = 0, SCK = 0, MISO = AIN for SPI Mode 0 & 1 (CPOL = 0) */
    /* MOSI = 0, SCK = 1, MISO = AIN for SPI Mode 2 & 3 (CPOL = 1) */
#if defined(SPI_NUMOF)
    for (i = 0; i < SPI_NUMOF; i++) {
        /* check if SPI is in use */
        if (is_periph_clk(spi_config[i].apbbus, spi_config[i].rccmask) == 1) {
            /* SPI CLK polarity */
            if (spi_config[i].dev->CR1 & (1<<1)) {
                gpio_init(spi_config[i].sclk_pin, GPIO_IN_PU);
            } else {
                gpio_init(spi_config[i].sclk_pin, GPIO_IN_PD);
            }

            gpio_init(spi_config[i].mosi_pin, GPIO_IN_PD);
            gpio_init(spi_config[i].miso_pin, GPIO_AIN);
        }
     }
#endif
}

static inline void _pm_after(void) {
    uint32_t i;
    GPIO_TypeDef *port;
      
    /* restore GPIO settings */
    for (i = 0; i < 8; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        
        if (cpu_check_address((char *)port)) {
            port->PUPDR = lpm_gpio_pupdr[i];
            port->OTYPER = lpm_gpio_otyper[i];
            port->MODER = lpm_gpio_moder[i];
            #if defined (STM32L1XX_HD) || defined (STM32L1XX_XL)
                port->BRR = lpm_gpio_brr[i];
            #endif
        } else {
            break;
        }
    }

#if defined(UART_NUMOF)
    /* restore USART clocks */
    for (i = 0; i < UART_NUMOF; i++) {
        if (lpm_usart[i]) {
            uart_config[i].dev->CR1 |= USART_CR1_UE;
        }
    }
#endif
}

static inline uint32_t _ewup_config(void)
{
    uint32_t tmp = 0;
    
#if defined(CPU_FAM_STM32L4)
    #if defined(PWR_CR3_EWUP8)
            tmp |= PWR_CR3_EWUP8;
    #endif
    #if defined(PWR_CR3_EWUP7)
        tmp |= PWR_CR3_EWUP7;
    #endif
    #if defined(PWR_CR3_EWUP6)
        tmp |= PWR_CR3_EWUP6;
    #endif
    #if defined(PWR_CR3_EWUP5)
        tmp |= PWR_CR3_EWUP5;
    #endif
    #if defined(PWR_CR3_EWUP4)
        tmp |= PWR_CR3_EWUP4;
    #endif
    #if defined(PWR_CR3_EWUP3)
        tmp |= PWR_CR3_EWUP3;
    #endif
    #if defined(PWR_CR3_EWUP2)
        tmp |= PWR_CR3_EWUP2;
    #endif
    #if defined(PWR_CR3_EWUP1)
        tmp |= PWR_CR3_EWUP1;
    #endif
#else
    #ifdef PM_EWUP_CONFIG
        tmp |= PM_EWUP_CONFIG;
    #elif defined(PWR_CSR_EWUP)
        tmp |= PWR_CSR_EWUP;
    #else
        #if defined(PWR_CSR_EWUP8)
            tmp |= PWR_CSR_EWUP8;
        #endif
        #if defined(PWR_CSR_EWUP7)
            tmp |= PWR_CSR_EWUP7;
        #endif
        #if defined(PWR_CSR_EWUP6)
            tmp |= PWR_CSR_EWUP6;
        #endif
        #if defined(PWR_CSR_EWUP5)
            tmp |= PWR_CSR_EWUP5;
        #endif
        #if defined(PWR_CSR_EWUP4)
            tmp |= PWR_CSR_EWUP4;
        #endif
        #if defined(PWR_CSR_EWUP3)
            tmp |= PWR_CSR_EWUP3;
        #endif
        #if defined(PWR_CSR_EWUP2)
            tmp |= PWR_CSR_EWUP2;
        #endif
        #if defined(PWR_CSR_EWUP1)
            tmp |= PWR_CSR_EWUP1;
        #endif
    #endif
#endif
    return tmp;
}

static uint8_t powermode;

enum pm_mode pm_set(enum pm_mode mode)
{
    int deep = 0;
    
    powermode = mode;
    
    switch (mode) {
        case PM_POWERDOWN:
#if defined(CPU_FAM_STM32L4)
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STANDBY;
#else
            /* Set PDDS to enter standby mode on deepsleep and clear flags */
            PWR->CR |= (PWR_CR_PDDS | PWR_CR_CWUF | PWR_CR_CSBF);
            /* Enable WKUP pin to use for wakeup from standby mode */
            /*
#if defined(CPU_FAM_STM32L0)
            PWR->CSR |= (PWR_CSR_EWUP1 | PWR_CSR_EWUP2);
#if !defined(CPU_LINE_STM32L053xx)
            */
            /* STM32L053 only have 2 wake pins */
            /*
            PWR->CSR |= PWR_CSR_EWUP3;
            */
#endif

#if defined(CPU_FAM_STM32L0) || defined (CPU_FAM_STM32L1)
            /* Disable Vrefint in standby mode */
            PWR->CR |= PWR_CR_ULP;
#endif

/* Enable WKUP pin to use for wakeup from standby mode */
#if defined(CPU_FAM_STM32L4)
            PWR->CR3 |= _ewup_config();
#else
            PWR->CSR |= _ewup_config();
#endif

            /* Set SLEEPDEEP bit of system control block */
            deep = 1;
            
            cortexm_sleep(deep);
            break;
        case PM_SLEEP:
#if defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1)
            /* Clear PDDS to enter stop mode on */
            PWR->CR &= ~(PWR_CR_PDDS);
            
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            
            /* Voltage regulator in LP mode */
            PWR->CR |= PWR_CR_LPSDSR;
            
            /* Disable Vrefint in stop mode */
            PWR->CR |= PWR_CR_ULP;
#if defined(CPU_FAM_STM32L0)
            /* set to 0 to select MSI as wakeup clock */
            /* set to 1 to select HSI16 */
            RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
#endif
#elif defined(CPU_FAM_STM32L4)
            /* Set STOP 0 mode by default */
            PWR->CR1 &= ~PWR_CR1_LPMS;
            PWR->CR1 |= PWR_CR1_LPMS_STOP0;
#else   /* STM32Fxxx series */
            /* Clear PDDS and LPDS bits to enter stop mode on */
            /* deepsleep with voltage regulator on */
            PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS);
            PWR->CR |= PM_STOP_CONFIG;
#endif
            /* Set SLEEPDEEP bit of system control block */
            SCB->SCR |=  (SCB_SCR_SLEEPDEEP_Msk);
            
            unsigned state = irq_disable();            
            _pm_before();
            
            #if defined (__CC_ARM)
                __force_stores();
            #endif

            __DSB();
            __WFI();
            
            /* Re-init clock after STOP */
            stmclk_init_sysclk();
            
            _pm_after();
            
            irq_restore(state);
            break;
        case PM_IDLE:
            break;
        case PM_ON:
            break;
        case PM_OFF:
            break;
        case PM_UNKNOWN:
            break;
    }

    if (deep) {
        /* Re-init clock after STOP */
        stmclk_init_sysclk();
    }

    return PM_UNKNOWN;
}

enum pm_mode pm_get(void) {
    return powermode;
}

void pm_init(void) {
    /* Nothing to do here yet */
}

void pm_off(void)
{
    irq_disable();
    pm_set(PM_OFF);
}

