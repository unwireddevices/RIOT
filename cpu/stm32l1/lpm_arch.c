/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2016 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @{
 *
 * @file
 * @brief       Implementation of the kernel's lpm interface
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Oleg Artamonov <info@unwds.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "arch/lpm_arch.h"

#include "stm32l1xx.h"
#include "xtimer.h"

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"

#define GPIO_LOW_POWER

#ifdef GPIO_LOW_POWER
static uint32_t lpm_gpio_moder[8];
static uint32_t lpm_gpio_pupdr[8];
static uint16_t lpm_gpio_otyper[8];
static uint32_t lpm_gpio_ospeedr[8];
static uint16_t lpm_gpio_odr[8];
static uint8_t  lpm_usart[5];
static uint32_t ahb_gpio_clocks;
static uint32_t tmpreg;

/* We are not using gpio_init as it sets GPIO clock speed to maximum */
static void pin_set(GPIO_TypeDef* port, uint8_t pin, uint8_t value){
    port->MODER &= ~(3 << (2*pin));
    port->MODER |= (1 << (2*pin));
    port->PUPDR &= ~(3 << (2*pin));
    port->OTYPER &= ~(1 << pin);
    if (value) {
        port->ODR |= (1 << pin);
    } else {
        port->ODR &= ~(1 << pin);
    }
}

/* put GPIOs in low-power state */
static void lpm_before_i_go_to_sleep (void) {
	/* Disable all USART interfaces in use */
	/* without it, RX will receive some garbage when MODER is changed */
memset(lpm_usart, 0, sizeof(lpm_usart));
#if UART_0_EN
    if (UART_0_ISON()) {
		UART_0_CLKDIS();
		lpm_usart[0] = 1;
    }
#endif
#if UART_1_EN
    if (UART_1_ISON()) {
		UART_1_CLKDIS();
        lpm_usart[1] = 1;
	}
#endif
#if UART_2_EN
    if (UART_2_ISON()) {
		UART_2_CLKDIS();
        lpm_usart[2] = 1;
    }
#endif
#if UART_3_EN
    if (UART_3_ISON()) {
		UART_3_CLKDIS();
        lpm_usart[3] = 1;
	}
#endif
#if UART_4_EN
    if (UART_4_ISON()) {
		UART_4_CLKDIS();
        lpm_usart[4] = 1;
    }
#endif

    /* save GPIO clock configuration */
    ahb_gpio_clocks = RCC->AHBENR & 0xFF;
    /* enable all GPIO clocks */
	periph_clk_en(AHB, 0xFF);
    
    uint8_t i;
    uint8_t p;
    uint32_t mask;
    GPIO_TypeDef *port;
    
    uint32_t addr_diff = GPIOB_BASE - GPIOA_BASE;
    uint32_t gpio_base_addr = 0;
    
    for (i = 0; i < CPU_NUMBER_OF_PORTS; i++) {
        gpio_base_addr = GPIOA_BASE + i*addr_diff;
        port = (GPIO_TypeDef *)gpio_base_addr;
        
        /* save GPIO registers values */
        lpm_gpio_moder[i] = port->MODER;
        lpm_gpio_pupdr[i] = port->PUPDR;
        lpm_gpio_otyper[i] = (port->OTYPER & 0xFFFF);
        lpm_gpio_ospeedr[i] = port->OSPEEDR;
        lpm_gpio_odr[i] = (port->ODR & 0xFFFF);
        
        mask = 0xFFFFFFFF;
        
        /* ignore GPIOs registered for external interrupts */
        /* they may be used as wakeup sources */
        for (p = 0; p < 16; p ++) {
            if (EXTI->IMR & (1 << p)) {
                if (((SYSCFG->EXTICR[p >> 2]) >> ((p & 0x03) * 4)) == i) {
                    mask &= ~((uint32_t)0x03 << (p*2));
                }
            }
        }

        /* disable pull-ups on GPIOs */
        tmpreg = port->PUPDR;
        tmpreg &= ~mask;
        port->PUPDR = tmpreg;
        
        /* set GPIOs to AIN mode */
        tmpreg = port->MODER;
        tmpreg |= mask;
        port->MODER = tmpreg;
        
        /* set lowest speed */
        port->OSPEEDR = 0;
    }
    
    /* specifically set GPIOs used for external SPI devices */
    /* NSS = 1, MOSI = 0, SCK = 0, MISO doesn't matter */
    /* NSS = 1, MOSI = 0, SCK = 0, MISO doesn't matter */
#if SPI_0_EN
    if (SPI_0_ISON()) {
        pin_set(SPI_0_PORT, SPI_0_PIN_NSS, 1);
        pin_set(SPI_0_PORT, SPI_0_PIN_SCK, 0);
        pin_set(SPI_0_PORT, SPI_0_PIN_MOSI, 0);
    }
#endif
#if SPI_1_EN
    if (SPI_1_ISON()) {
        pin_set(SPI_1_PORT, SPI_1_PIN_NSS, 1);
        pin_set(SPI_1_PORT, SPI_1_PIN_SCK, 0);
        pin_set(SPI_1_PORT, SPI_1_PIN_MOSI, 0);
    }
#endif
#if SPI_2_EN
    if (SPI_2_ISON()) {
        pin_set(SPI_2_PORT, SPI_2_PIN_NSS, 1);
        pin_set(SPI_2_PORT, SPI_2_PIN_SCK, 0);
        pin_set(SPI_2_PORT, SPI_2_PIN_MOSI, 0);
    }
#endif

    /* set UART TX pin to 1 */
#if UART_0_EN
    if (UART_0_ISON()) {
        pin_set((GPIO_TypeDef *)(UART_0_TX_PIN & ~(0x0f)), UART_0_TX_PIN & 0x0f, 1);
    }
#endif
#if UART_1_EN
    if (UART_1_ISON()) {
        pin_set((GPIO_TypeDef *)(UART_1_TX_PIN & ~(0x0f)), UART_1_TX_PIN & 0x0f, 1);
	}
#endif
#if UART_2_EN
    if (UART_2_ISON()) {
        pin_set((GPIO_TypeDef *)(UART_2_TX_PIN & ~(0x0f)), UART_2_TX_PIN & 0x0f, 1);
    }
#endif
#if UART_3_EN
    if (UART_3_ISON()) {
        pin_set((GPIO_TypeDef *)(UART_3_TX_PIN & ~(0x0f)), UART_3_TX_PIN & 0x0f, 1);
	}
#endif
#if UART_4_EN
    if (UART_4_ISON()) {
        pin_set((GPIO_TypeDef *)(UART_4_TX_PIN & ~(0x0f)), UART_4_TX_PIN & 0x0f, 1);
    }
#endif

    /* restore GPIO clocks */
    tmpreg = RCC->AHBENR;
    tmpreg &= ~((uint32_t)0xFF);
    tmpreg |= ahb_gpio_clocks;
	periph_clk_en(AHB, tmpreg);
}


/* restore GPIO settings */
static void lpm_when_i_wake_up (void) {
    /* enable all GPIO clocks */
	periph_clk_en(AHB, 0xFF);
    
    uint8_t i;
    GPIO_TypeDef *port;
    uint32_t addr_diff = GPIOB_BASE - GPIOA_BASE;
    uint32_t gpio_base_addr = 0;
	  
    /* restore GPIO settings */
    for (i = 0; i < CPU_NUMBER_OF_PORTS; i++) {
        gpio_base_addr = GPIOA_BASE + i*addr_diff;
        port = (GPIO_TypeDef *)gpio_base_addr;
        
		port->PUPDR = lpm_gpio_pupdr[i];
		port->OTYPER = lpm_gpio_otyper[i];
        port->OSPEEDR = lpm_gpio_ospeedr[i];
        port->ODR = lpm_gpio_odr[i];
        port->MODER = lpm_gpio_moder[i];
    }

    /* restore GPIO clocks */
    tmpreg = RCC->AHBENR;
    tmpreg &= ~((uint32_t)0xFF);
    tmpreg |= ahb_gpio_clocks;
    periph_clk_en(AHB, tmpreg);
	
	/* restore USART clocks */
#if UART_0_EN
	if (lpm_usart[0]) { UART_0_CLKEN(); };
#endif
#if UART_1_EN
	if (lpm_usart[1]) { UART_1_CLKEN(); };
#endif
#if UART_2_EN
	if (lpm_usart[2]) { UART_2_CLKEN(); };
#endif
#if UART_3_EN
	if (lpm_usart[3]) { UART_3_CLKEN(); };
#endif
#if UART_4_EN
	if (lpm_usart[4]) { UART_4_CLKEN(); };
#endif
}
#endif

void lpm_arch_init(void)
{
	/* Unlock the RUN_PD bit to change flash settings */  
	FLASH->PDKEYR = FLASH_PDKEY1;
	FLASH->PDKEYR = FLASH_PDKEY2;
	/* Enable flash power down during sleep */
	FLASH->ACR |= FLASH_ACR_SLEEP_PD;
	
    /* Disable peripherals in Sleep mode */
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM2LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM3LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM4LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM5LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM6LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_TIM7LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_LCDLPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_WWDGLPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_SPI2LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_SPI3LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_USART2LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_USART3LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_UART4LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_UART5LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_I2C1LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_I2C2LPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_USBLPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_PWRLPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_DACLPEN);
    RCC->APB1LPENR &= ~(RCC_APB1LPENR_COMPLPEN);

    RCC->APB2LPENR &= ~(RCC_APB2LPENR_SYSCFGLPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM9LPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM10LPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_TIM11LPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_ADC1LPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_SDIOLPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_SPI1LPEN);
    RCC->APB2LPENR &= ~(RCC_APB2LPENR_USART1LPEN);

    RCC->AHBLPENR &= ~(RCC_AHBLPENR_CRCLPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_FLITFLPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_SRAMLPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_DMA1LPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_DMA2LPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_AESLPEN);
    RCC->AHBLPENR &= ~(RCC_AHBLPENR_FSMCLPEN);
    
    /* disable only GPIO ports which do not have IRQs associated */
	/* SEEMS WE DO NOT NEED CLOCK RUNNING FOR EXT IRQ IN SLEEP MODE */
	/*
    uint8_t port;
    uint8_t pin;
    uint8_t is_irq_enabled;
    for (port = 0; port < 8; port++) {
        is_irq_enabled = 0;
        for (pin = 0; pin < 16; pin ++) {
            if (EXTI->IMR & (1 << pin)) {
                if (((SYSCFG->EXTICR[pin >> 2]) >> ((pin & 0x03) * 4)) == port) {
                    is_irq_enabled = 1;
                }
            }
        }
        if (is_irq_enabled) {
            RCC->AHBLPENR &= ~(1 << port);
        }
    }
	*/
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
    switch (target) {
        case LPM_SLEEP:               /* Low-power sleep mode */
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Enable low-power mode of the voltage regulator */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Clear SLEEPDEEP bit */
            SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP);

            irq_disable();
            
#ifdef GPIO_LOW_POWER
            lpm_before_i_go_to_sleep();
#endif

            /* Switch to 65kHz clock */
            switch_to_msi(RCC_ICSCR_MSIRANGE_0, RCC_CFGR_HPRE_DIV1);

            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
			
            /* Switch back to default speed */
			clk_init();

#ifdef GPIO_LOW_POWER
            lpm_when_i_wake_up();
#endif
			
            irq_enable();
            break;

        case LPM_POWERDOWN:         /* STOP mode */
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Regulator in LP mode */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Enable Ultra Low Power mode */
            PWR->CR |= PWR_CR_ULP;
            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP;

            irq_disable();

#ifdef GPIO_LOW_POWER
            lpm_before_i_go_to_sleep();
#endif
            
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();

            /* Clear SLEEPDEEP bit */
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP);
			
			/* Restore clocks and PLL */
			/* (MCU is running on MSI clock after STOP) */
			clk_init();
            
            /* Wait for the reference voltage */
            /* while(!(PWR->CSR & PWR_CSR_VREFINTRDYF)) {} */
			
#ifdef GPIO_LOW_POWER
            lpm_when_i_wake_up();
#endif
            
            irq_enable();

            break;

        case LPM_OFF:               /* Standby mode */
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
        
            /* Select STANDBY mode */
            PWR->CR |= PWR_CR_PDDS;

            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP;

            /* Enable Ultra Low Power mode */
            PWR->CR |= PWR_CR_ULP;

            /* This option is used to ensure that store operations are completed */
            #if defined (__CC_ARM)
            __force_stores();
            #endif
			
			__disable_irq();
			
            /* Request Wait For Interrupt */
			__DSB();
            __WFI();
            break;

        /* do nothing here */
        case LPM_UNKNOWN:
        case LPM_ON:
        case LPM_IDLE:
        default:
            break;
    }

    return 0;
}

enum lpm_mode lpm_arch_get(void)
{
    /* TODO */
    return 0;
}

void lpm_arch_awake(void)
{
    /* Disable Ultra Low Power mode */
    PWR->CR &= ~PWR_CR_ULP;

    PWR->CR &= ~((uint32_t)PWR_CR_LPRUN);
    PWR->CR &= ~((uint32_t)PWR_CR_LPSDSR);
}

void lpm_arch_begin_awake(void)
{
    /* TODO */
}

void lpm_arch_end_awake(void)
{
    /* TODO */
}
