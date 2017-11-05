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

#include "lpm.h"
#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static uint32_t lpm_gpio_moder[8];
static uint32_t lpm_gpio_pupdr[8];
static uint16_t lpm_gpio_otyper[8];
static uint16_t lpm_gpio_odr[8];
static uint8_t  lpm_usart[UART_NUMOF];
static uint32_t ahb_gpio_clocks;
static uint32_t tmpreg;

static uint16_t lpm_portmask_system[8] = { 0 };
static uint16_t lpm_portmask_user[8] = { 0 };

volatile int lpm_run_mode;

/* We are not using gpio_init as it sets GPIO clock speed to maximum */
/* We add GPIOs we touched to exclusion mask lpm_portmask_system */
static void pin_set(GPIO_TypeDef* port, uint8_t pin, uint8_t value) {
    tmpreg = port->MODER;
    tmpreg &= ~(3 << (2*pin));
    tmpreg |= (1 << (2*pin));
    port->MODER = tmpreg;

    port->PUPDR &= ~(3 << (2*pin));
    port->OTYPER &= ~(1 << pin);
    if (value) {
        port->ODR |= (1 << pin);
    } else {
        port->ODR &= ~(1 << pin);
    }
    
    lpm_portmask_system[((uint32_t)port >> 10) & 0x0f] |= 1 << pin;
}

/* put GPIOs in low-power state */
static void lpm_before_i_go_to_sleep (void) {
	uint8_t i;
    uint8_t p;
    uint32_t mask;
    GPIO_TypeDef *port;
	
	for (i = 0; i < cpu_ports_number; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));

        /* save GPIO registers values */
        lpm_gpio_moder[i] = port->MODER;
        lpm_gpio_pupdr[i] = port->PUPDR;
        lpm_gpio_otyper[i] = (uint16_t)(port->OTYPER & 0xFFFF);
        lpm_gpio_odr[i] = (uint16_t)(port->ODR & 0xFFFF);
	}
	
    /* Disable all USART interfaces in use */
    /* without it, RX will receive some garbage when MODER is changed */

    for (i = 0; i < UART_NUMOF; i++) {
        if (uart_config[i].dev->CR1 & USART_CR1_UE) {
            uart_config[i].dev->CR1 &= ~USART_CR1_UE;
            pin_set((GPIO_TypeDef *)(uart_config[i].tx_pin & ~(0x0f)), uart_config[i].tx_pin & 0x0f, 1);
            lpm_usart[i] = 1;
        } else {
            lpm_usart[i] = 0;
        }
    }

    /* specifically set GPIOs used for external SPI devices */
    /* NSS = 1, MOSI = 0, SCK = 0, MISO doesn't matter */
    /* NSS = 1, MOSI = 0, SCK = 0, MISO doesn't matter */
#if SPI_0_EN
    if (SPI_0_ISON()) {
        pin_set(SPI_0_PORT, SPI_0_PIN_NSS, 1);
        pin_set(SPI_0_PORT, SPI_0_PIN_SCK, 0);
        pin_set(SPI_0_PORT, SPI_0_PIN_MOSI, 0);
    } else {
		p = ((uint32_t)SPI_0_PORT >> 10) & 0x0f;
		lpm_portmask_system[p] &= ~(1 << SPI_0_PIN_NSS);
		lpm_portmask_system[p] &= ~(1 << SPI_0_PIN_SCK);
		lpm_portmask_system[p] &= ~(1 << SPI_0_PIN_MOSI);
	}
#endif
#if SPI_1_EN
    if (SPI_1_ISON()) {
        pin_set(SPI_1_PORT, SPI_1_PIN_NSS, 1);
        pin_set(SPI_1_PORT, SPI_1_PIN_SCK, 0);
        pin_set(SPI_1_PORT, SPI_1_PIN_MOSI, 0);
    } else {
		p = ((uint32_t)SPI_1_PORT >> 10) & 0x0f;
		lpm_portmask_system[p] &= ~(1 << SPI_1_PIN_NSS);
		lpm_portmask_system[p] &= ~(1 << SPI_1_PIN_SCK);
		lpm_portmask_system[p] &= ~(1 << SPI_1_PIN_MOSI);
	}
#endif
#if SPI_2_EN
    if (SPI_2_ISON()) {
        pin_set(SPI_2_PORT, SPI_2_PIN_NSS, 1);
        pin_set(SPI_2_PORT, SPI_2_PIN_SCK, 0);
        pin_set(SPI_2_PORT, SPI_2_PIN_MOSI, 0);
    } else {
		p = ((uint32_t)SPI_2_PORT >> 10) & 0x0f;
		lpm_portmask_system[p] &= ~(1 << SPI_2_PIN_NSS);
		lpm_portmask_system[p] &= ~(1 << SPI_2_PIN_SCK);
		lpm_portmask_system[p] &= ~(1 << SPI_2_PIN_MOSI);
	}
#endif

    /* save GPIO clock configuration */
    ahb_gpio_clocks = RCC->AHBENR & 0xFF;
    /* enable all GPIO clocks */
    periph_clk_en(AHB, 0xFF);
    
    for (i = 0; i < cpu_ports_number; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        mask = 0xFFFFFFFF;
        if (EXTI->IMR) {
            for (p = 0; p < 16; p ++) {
                /* exclude GPIOs we previously set with pin_set */
                if ((lpm_portmask_system[i] | lpm_portmask_user[i]) & (1 << p)) {
                    mask &= ~(0x03 << (p*2));
                } else {
                    if ((EXTI->IMR & (1 << p)) && ((((SYSCFG->EXTICR[p >> 2]) >> ((p & 0x03) * 4)) & 0xF) == i)) {
                        mask &= ~(0x03 << (p*2));
                    }
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
    }

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
      
    /* restore GPIO settings */
    for (i = 0; i < cpu_ports_number; i++) {
        port = (GPIO_TypeDef *)(GPIOA_BASE + i*(GPIOB_BASE - GPIOA_BASE));
        
        port->PUPDR = lpm_gpio_pupdr[i];
        port->OTYPER = lpm_gpio_otyper[i];
        port->ODR = lpm_gpio_odr[i];
        port->MODER = lpm_gpio_moder[i];
    }

    /* restore GPIO clocks */
    tmpreg = RCC->AHBENR;
    tmpreg &= ~((uint32_t)0xFF);
    tmpreg |= ahb_gpio_clocks;
    periph_clk_en(AHB, tmpreg);
    
    /* restore USART clocks */
    for (i = 0; i < UART_NUMOF; i++) {
        if (lpm_usart[i]) {
            uart_config[i].dev->CR1 |= USART_CR1_UE;
        }
    }
}

/* Do not change GPIO state in sleep mode */
void lpm_arch_add_gpio_exclusion(gpio_t gpio) {
	uint8_t port = ((uint32_t)gpio >> 10) & 0x0f;
	uint8_t pin = ((uint32_t)gpio & 0x0f);
	
	lpm_portmask_user[port] |= (uint16_t)(1<<pin);
}

/* Change GPIO state to AIN in sleep mode */
void lpm_arch_del_gpio_exclusion(gpio_t gpio) {
	uint8_t port = ((uint32_t)gpio >> 10) & 0x0f;
	uint8_t pin = ((uint32_t)gpio & 0x0f);
	
	lpm_portmask_user[port] &= ~(uint16_t)(1<<pin);
}

/* Select CPU clocking between default (LPM_ON) and medium-speed (LPM_IDLE) */
void lpm_select_run_mode(uint8_t lpm_mode) {
	switch(lpm_run_mode) {
		case LPM_ON:
            DEBUG("Switching to LPM_ON");
			clk_init();
			break;
		case LPM_IDLE:
            DEBUG("Switching to LPM_IDLE");
            /* 115200 bps stdio UART with default 16x oversamplig needs 2 MHz or 4 MHz MSI clock */
            /* at 1 MHz, it will be switched to 8x oversampling with 3.55 % baudrate error */
            /* if you need stdio UART at lower frequencies, change its settings to lower baudrate */
			switch_to_msi(RCC_ICSCR_MSIRANGE_5, RCC_CFGR_HPRE_DIV1);
			break;
		default:
            DEBUG("Switching to LPM_IDLE");
			clk_init();
		break;
	}
    
    /* Recalculate xtimer frequency */
    /* NB: default XTIMER_HZ clock is 1 MHz, so CPU clock must be at least 1 MHz for xtimer to work properly */
    xtimer_init();
    
    /* Recalculate stdio UART baudrate */
    uart_set_baudrate(UART_STDIO_DEV, UART_STDIO_BAUDRATE);
}

void lpm_arch_init(void)
{
	lpm_run_mode = LPM_ON;
	
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
}

enum lpm_mode lpm_arch_set(enum lpm_mode target)
{
    switch (target) {
        case LPM_SLEEP:               /* Low-power sleep mode */
            DEBUG("Switching to LPM_SLEEP");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Enable low-power mode of the voltage regulator */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Clear SLEEPDEEP bit */
            SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP);

            irq_disable();
            
            lpm_before_i_go_to_sleep();

            /* Switch to 65kHz clock */
            switch_to_msi(RCC_ICSCR_MSIRANGE_0, RCC_CFGR_HPRE_DIV1);

            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
            
            /* Switch back to default speed */
			lpm_select_run_mode(lpm_run_mode);

            lpm_when_i_wake_up();
          
            irq_enable();
            break;

        case LPM_POWERDOWN:         /* STOP mode */
            DEBUG("Switching to LPM_POWERDOWN");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Regulator in LP mode */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Enable Ultra Low Power mode */
            PWR->CR |= PWR_CR_ULP;
            /* Set SLEEPDEEP bit of Cortex System Control Register */
            SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP;
            
            /* Enable debug in STOP mode */
            /* DBGMCU->CR |= DBGMCU_CR_DBG_STOP; */

            irq_disable();

            lpm_before_i_go_to_sleep();
           
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();

            /* Clear SLEEPDEEP bit */
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP);
            
            /* Restore clocks and PLL */
            /* (MCU is running on MSI clock after STOP) */
			lpm_select_run_mode(lpm_run_mode);
            
            lpm_when_i_wake_up();

			irq_enable();

            break;

        case LPM_OFF:               /* Standby mode */
            DEBUG("Switching to LPM_OFF");
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
            
            irq_disable();
            
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
            break;

        case LPM_UNKNOWN:
			break;
        case LPM_ON:
            DEBUG("Switching to LPM_ON");
			irq_disable();
			lpm_run_mode = LPM_ON;
			lpm_select_run_mode(lpm_run_mode);
			irq_enable();
			break;
        case LPM_IDLE:
            if (!lpm_prevent_switch) {
                DEBUG("Switching to LPM_IDLE");
                irq_disable();
                lpm_run_mode = LPM_IDLE;
                lpm_select_run_mode(lpm_run_mode);
                irq_enable();
            }
			break;
        default:
            break;
    }

    return 0;
}

enum lpm_mode lpm_arch_get(void)
{
    return lpm_run_mode;
}

void lpm_arch_awake(void)
{
    /* TODO */
}

void lpm_arch_begin_awake(void)
{
    /* TODO */
}

void lpm_arch_end_awake(void)
{
    /* TODO */
}
