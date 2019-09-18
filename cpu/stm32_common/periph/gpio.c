/*
 * Copyright (C) 2014-2015 Freie Universität Berlin
 *               2015 Hamburg University of Applied Sciences
 *               2017 Inria
 *               2017 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_gpio
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Katja Kirstein <katja.kirstein@haw-hamburg.de>
 * @author      Vincent Dupont <vincent@otakeys.com>
 *
 * @}
 */


#include "cpu.h"
#include "periph/gpio.h"
#include "periph_conf.h"

/* this implementation is not valid for the stm32f1 */
#ifndef CPU_FAM_STM32F1

#ifdef MODULE_PERIPH_GPIO_IRQ
/**
 * @brief   The STM32F0 family has 16 external interrupt lines
 */
#define EXTI_NUMOF          (16U)

/**
 * @brief   Allocate memory for one callback and argument per EXTI channel
 */
static gpio_isr_ctx_t isr_ctx[EXTI_NUMOF];
#endif /* MODULE_PERIPH_GPIO_IRQ */

/**
 * @brief   Extract the port base address from the given pin identifier
 */
static inline GPIO_TypeDef *_port(gpio_t pin)
{
    return (GPIO_TypeDef *)(pin & ~(0x0f));
}

/**
 * @brief   Extract the port number form the given identifier
 *
 * The port number is extracted by looking at bits 10, 11, 12, 13 of the base
 * register addresses.
 */
static inline int _port_num(gpio_t pin)
{
    return ((pin >> 10) & 0x0f);
}

/**
 * @brief   Extract the pin number from the last 4 bit of the pin identifier
 */
static inline int _pin_num(gpio_t pin)
{
    return (pin & 0x0f);
}

int gpio_init(gpio_t pin, gpio_mode_t mode)
{
    if (pin == GPIO_UNDEF) {
        return -1;
    }

    GPIO_TypeDef *port = _port(pin);
    int pin_num = _pin_num(pin);

    /* enable clock */
#if defined(CPU_FAM_STM32F0) || defined (CPU_FAM_STM32F3) || defined(CPU_FAM_STM32L1)
    periph_clk_en(AHB, (RCC_AHBENR_GPIOAEN << _port_num(pin)));
#elif defined (CPU_FAM_STM32L0)
    periph_clk_en(IOP, (RCC_IOPENR_GPIOAEN << _port_num(pin)));
#elif defined (CPU_FAM_STM32L4)
    periph_clk_en(AHB2, (RCC_AHB2ENR_GPIOAEN << _port_num(pin)));
#ifdef PWR_CR2_IOSV
    if (port == GPIOG) {
        /* Port G requires external power supply */
        periph_clk_en(APB1, RCC_APB1ENR1_PWREN);
        PWR->CR2 |= PWR_CR2_IOSV;
    }
#endif /* PWR_CR2_IOSV */
#else
    periph_clk_en(AHB1, (RCC_AHB1ENR_GPIOAEN << _port_num(pin)));
#endif

    uint32_t irqs = irq_disable();

    /* using temporary variable to prevent glitches when switching modes */
    uint32_t tmpreg;

    /* set mode */
    tmpreg = port->MODER;
    tmpreg &= ~(0x3 << (2 * pin_num));
    tmpreg |=  ((mode & 0x3) << (2 * pin_num));
    port->MODER = tmpreg;

    /* set pull resistor configuration */
    tmpreg = port->PUPDR;
    tmpreg &= ~(0x3 << (2 * pin_num));
    tmpreg |=  (((mode >> 2) & 0x3) << (2 * pin_num));
    port->PUPDR = tmpreg;

    /* set output mode */
    tmpreg = port->OTYPER;
    tmpreg &= ~(1 << pin_num);
    tmpreg |=  (((mode >> 4) & 0x1) << pin_num);
    port->OTYPER = tmpreg;

    /* set pin speed to maximum */
    port->OSPEEDR |= (3 << (2 * pin_num));
#if defined (STM32L1XX_HD) || defined (STM32L1XX_XL)
    port->BRR = (1 << pin_num);
#endif

    irq_restore(irqs);

    return 0;
}

void gpio_init_af(gpio_t pin, gpio_af_t af)
{
    if (pin == GPIO_UNDEF) {
        return;
    }

    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    uint32_t irqs = irq_disable();

    /* set pin to AF mode */
    /* using temporary variable to prevent glitches when switching mode */
    uint32_t tmpreg;
    tmpreg = port->MODER;
    tmpreg &= ~(3 << (2 * pin_num));
    tmpreg |= (2 << (2 * pin_num));
    port->MODER = tmpreg;
    /* set selected function */
    tmpreg = port->AFR[(pin_num > 7) ? 1 : 0];
    tmpreg &= ~(0xf << ((pin_num & 0x07) * 4));
    tmpreg |= (af << ((pin_num & 0x07) * 4));
    port->AFR[(pin_num > 7) ? 1 : 0] = tmpreg;

    irq_restore(irqs);
}

void gpio_init_analog(gpio_t pin)
{
    if (pin == GPIO_UNDEF) {
        return;
    }

    /* enable clock, needed as this function can be used without calling
     * gpio_init first */
#if defined(CPU_FAM_STM32F0) || defined (CPU_FAM_STM32F3) || defined(CPU_FAM_STM32L1)
    periph_clk_en(AHB, (RCC_AHBENR_GPIOAEN << _port_num(pin)));
#elif defined (CPU_FAM_STM32L0)
    periph_clk_en(IOP, (RCC_IOPENR_GPIOAEN << _port_num(pin)));
#elif defined (CPU_FAM_STM32L4)
    periph_clk_en(AHB2, (RCC_AHB2ENR_GPIOAEN << _port_num(pin)));
#else
    periph_clk_en(AHB1, (RCC_AHB1ENR_GPIOAEN << _port_num(pin)));
#endif
    /* set to analog mode */
    _port(pin)->MODER |= (0x3 << (2 * _pin_num(pin)));
}

void gpio_irq_enable(gpio_t pin)
{
    if (pin == GPIO_UNDEF) {
        return;
    }

    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);

    /* check if IRQ is actually configured for this pin and port */
    if (!((SYSCFG->EXTICR[pin_num >> 2] & (0xf << ((pin_num & 0x03) * 4))) ^ (port_num << ((pin_num & 0x03) * 4)))) {
        EXTI->IMR |= (1 << _pin_num(pin));
    }
}

void gpio_irq_disable(gpio_t pin)
{
    if (pin == GPIO_UNDEF) {
        return;
    }

    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);

    /* check if IRQ is actually configured for this pin and port */
    if (!((SYSCFG->EXTICR[pin_num >> 2] & (0xf << ((pin_num & 0x03) * 4))) ^ (port_num << ((pin_num & 0x03) * 4)))) {
        EXTI->IMR &= ~(1 << pin_num);
    }
}

int gpio_read(gpio_t pin)
{
    if (pin == GPIO_UNDEF) {
        return -2;
    }

    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    uint8_t port_mode = (port->MODER & (3 << (pin_num * 2))) >> (pin_num * 2);

    if (port_mode == 1) {   /* if configured as output */
        return (port->ODR & (1 << pin_num)) >> pin_num;      /* read output data reg */
    }
    if (port_mode == 0) {
        return (port->IDR & (1 << pin_num)) >> pin_num;      /* else read input data reg */
    }

    /* configured as AF or AIN */
    return -1;
}

int gpio_get_status(gpio_t pin) {
    if (pin == GPIO_UNDEF) {
        return -1;
    }

    GPIO_TypeDef *port = _port(pin);
    uint32_t pin_num = _pin_num(pin);

    return (port->MODER & (3 << (pin_num * 2))) >> (pin_num * 2);
}

void gpio_set(gpio_t pin)
{
    if (pin != GPIO_UNDEF) {
        _port(pin)->BSRR = (1 << _pin_num(pin));
    }
}

void gpio_clear(gpio_t pin)
{
    if (pin != GPIO_UNDEF) {
        _port(pin)->BSRR = (1 << (_pin_num(pin) + 16));
    }
}

void gpio_toggle(gpio_t pin)
{
    if (gpio_read(pin)) {
        gpio_clear(pin);
    } else {
        gpio_set(pin);
    }
}

void gpio_write(gpio_t pin, int value)
{   
    if (value) {
        gpio_set(pin);
    } else {
        gpio_clear(pin);
    }
}

#ifdef MODULE_PERIPH_GPIO_IRQ
int gpio_init_int(gpio_t pin, gpio_mode_t mode, gpio_flank_t flank,
                  gpio_cb_t cb, void *arg)
{
    int pin_num = _pin_num(pin);
    int port_num = _port_num(pin);

    /* set callback */
    isr_ctx[pin_num].cb = cb;
    isr_ctx[pin_num].arg = arg;

    /* enable clock of the SYSCFG module for EXTI configuration */
#ifdef CPU_FAN_STM32F0
    periph_clk_en(APB2, RCC_APB2ENR_SYSCFGCOMPEN);
#else
    periph_clk_en(APB2, RCC_APB2ENR_SYSCFGEN);
#endif

    /* initialize pin as input */
    gpio_init(pin, mode);

    /* enable global pin interrupt */
#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32L0)
    if (pin_num < 2) {
        NVIC_EnableIRQ(EXTI0_1_IRQn);
    }
    else if (pin_num < 4) {
        NVIC_EnableIRQ(EXTI2_3_IRQn);
    }
    else {
        NVIC_EnableIRQ(EXTI4_15_IRQn);
    }
#else
    if (pin_num < 5) {
        NVIC_EnableIRQ(EXTI0_IRQn + pin_num);
    }
    else if (pin_num < 10) {
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    else {
        NVIC_EnableIRQ(EXTI15_10_IRQn);
    }
#endif
    /* configure the active flank */
    EXTI->RTSR &= ~(1 << pin_num);
    EXTI->RTSR |=  ((flank & 0x1) << pin_num);
    EXTI->FTSR &= ~(1 << pin_num);
    EXTI->FTSR |=  ((flank >> 1) << pin_num);
    /* enable specific pin as exti sources */
    SYSCFG->EXTICR[pin_num >> 2] &= ~(0xf << ((pin_num & 0x03) * 4));
    SYSCFG->EXTICR[pin_num >> 2] |= (port_num << ((pin_num & 0x03) * 4));

    /* clear any pending requests */
    EXTI->PR = (1 << pin_num);
    /* unmask the pins interrupt channel */
    EXTI->IMR |= (1 << pin_num);

    return 0;
}
void isr_exti(void)
{
    /* only generate interrupts against lines which have their IMR set */
    uint32_t pending_isr = (EXTI->PR & EXTI->IMR);
    for (size_t i = 0; i < EXTI_NUMOF; i++) {
        if (pending_isr & (1 << i)) {
            EXTI->PR = (1 << i);        /* clear by writing a 1 */
            isr_ctx[i].cb(isr_ctx[i].arg);
        }
    }
    cortexm_isr_end();
}
#endif /* MODULE_PERIPH_GPIO_IRQ */

#else
typedef int dont_be_pedantic;
#endif
