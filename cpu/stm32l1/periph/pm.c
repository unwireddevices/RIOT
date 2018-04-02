/*
 * Copyright (C) 2017 Unwired Devices LLC
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cortexm_common
 * @ingroup     drivers_periph_pm
 * @{
 *
 * @file
 * @brief       common periph/pm functions
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#include "cpu.h"
#include "periph/pm.h"

#include "cpu.h"
#include "board.h"
#include "periph_conf.h"
#include "periph/uart.h"
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

volatile int pm_prevent_sleep = 0;
volatile int pm_prevent_switch = 0;

/*
static uint32_t pm_gpio_moder[8];
static uint32_t pm_gpio_pupdr[8];
static uint16_t pm_gpio_otyper[8];
static uint16_t pm_gpio_odr[8];
static uint8_t  pm_usart[UART_NUMOF];
static uint32_t ahb_gpio_clocks;
static uint32_t tmpreg;

static uint16_t pm_portmask_system[8] = { 0 };
static uint16_t pm_portmask_user[8] = { 0 };
*/
volatile int pm_run_mode;

/* We are not using gpio_init as it sets GPIO clock speed to maximum */
/* We add GPIOs we touched to exclusion mask pm_portmask_system */
/*
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
    
    pm_portmask_system[((uint32_t)port >> 10) & 0x0f] |= 1 << pin;
}
*/

/* put GPIOs in low-power state */
static void pm_before_i_go_to_sleep (void) {
    /* seems nothing is needed here */
}


/* restore GPIO settings */
static void pm_when_i_wake_up (void) {
    /* seems nothing is needed here */
}

/* Do not change GPIO state in sleep mode */
void pm_add_gpio_exclusion(gpio_t gpio) {
	(void)gpio;
}

/* Change GPIO state to AIN in sleep mode */
void pm_del_gpio_exclusion(gpio_t gpio) {
	(void)gpio;
}

/* Select CPU clocking between default (PM_ON) and medium-speed (PM_IDLE) */
static void pm_select_run_mode(uint8_t pm_mode) {
	switch(pm_mode) {
		case PM_ON:
            DEBUG("Switching to PM_ON");
			clk_init();
			break;
		case PM_IDLE:
            DEBUG("Switching to PM_IDLE");
            /* 115200 bps stdio UART with default 16x oversamplig needs 2 MHz or 4 MHz MSI clock */
            /* at 1 MHz, it will be switched to 8x oversampling with 3.55 % baudrate error */
            /* if you need stdio UART at lower frequencies, change its settings to lower baudrate */
			switch_to_msi(RCC_ICSCR_MSIRANGE_5, RCC_CFGR_HPRE_DIV1);
			break;
		default:
            DEBUG("Switching to PM_IDLE");
			clk_init();
		break;
	}
    
#if defined(XTIMER_PRESENT)
    /* Recalculate xtimer frequency */
    /* NB: default XTIMER_HZ clock is 1 MHz, so CPU clock must be at least 1 MHz for xtimer to work properly */
    xtimer_init();
#endif
    
#if defined(UART_STDIO_DEV)
    /* Recalculate stdio UART baudrate */
    uart_set_baudrate(UART_STDIO_DEV, UART_STDIO_BAUDRATE);
#endif
}

void pm_set_lowest(void)
{
    if (!pm_prevent_sleep) {
        pm_set(PM_POWERDOWN);
    }
}

void pm_init(void)
{
    pm_run_mode = PM_ON;
	
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

enum pm_mode pm_set(enum pm_mode target)
{
    switch (target) {
        case PM_SLEEP:               /* Low-power sleep mode */
            DEBUG("Switching to PM_SLEEP");
            /* Clear Wakeup flag */    
            PWR->CR |= PWR_CR_CWUF;
            /* Enable low-power mode of the voltage regulator */
            PWR->CR |= PWR_CR_LPSDSR;
            /* Clear SLEEPDEEP bit */
            SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP);

            irq_disable();
            
            pm_before_i_go_to_sleep();

            /* Switch to 65kHz clock */
            switch_to_msi(RCC_ICSCR_MSIRANGE_0, RCC_CFGR_HPRE_DIV1);

            /* Request Wait For Interrupt */
            __DSB();
            __WFI();
            
            /* Switch back to default speed */
			pm_select_run_mode(pm_run_mode);

            pm_when_i_wake_up();
          
            irq_enable();
            break;

        case PM_POWERDOWN:         /* STOP mode */
            DEBUG("Switching to PM_POWERDOWN");
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

            pm_before_i_go_to_sleep();
           
            /* Request Wait For Interrupt */
            __DSB();
            __WFI();

            /* Clear SLEEPDEEP bit */
            SCB->SCR &= (uint32_t) ~((uint32_t)SCB_SCR_SLEEPDEEP);
            
            /* Restore clocks and PLL */
            /* (MCU is running on MSI clock after STOP) */
			pm_select_run_mode(pm_run_mode);
            
            pm_when_i_wake_up();

			irq_enable();

            break;

        case PM_OFF:               /* Standby mode */
            DEBUG("Switching to PM_OFF");
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

        case PM_UNKNOWN:
			break;
        case PM_ON:
            DEBUG("Switching to PM_ON");
			irq_disable();
			pm_run_mode = PM_ON;
			pm_select_run_mode(pm_run_mode);
			irq_enable();
			break;
        case PM_IDLE:
            if (!pm_prevent_switch) {
                DEBUG("Switching to PM_IDLE");
                irq_disable();
                pm_run_mode = PM_IDLE;
                pm_select_run_mode(pm_run_mode);
                irq_enable();
            }
			break;
        default:
            break;
    }

    return 0;
}

enum pm_mode pm_get(void)
{
    return pm_run_mode;
}