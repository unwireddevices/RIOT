/*
 * Copyright (C) 2015 Jan Wagner <mail@jwagner.eu>
 *               2015-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf5x_common
 * @ingroup     drivers_periph_gpio
 * @{
 *
 * @file
 * @brief       Low-level GPIO driver implementation
 *
 * @note        This GPIO driver implementation supports only one pin to be
 *              defined as external interrupt.
 *
 * @author      Christian Kühling <kuehling@zedat.fu-berlin.de>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Jan Wagner <mail@jwagner.eu>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include "cpu.h"
#include "periph/gpio.h"
#include "periph_cpu.h"
#include "periph_conf.h"

#include "assert.h"

#define PORT_BIT            (1 << 5)
#define PIN_MASK            (0x1f)

/**
 * @brief   Place to store the interrupt context
 */
static gpio_isr_ctx_t exti_chan;

/**
 * @brief   Get the port's base address
 */
static inline NRF_GPIO_Type* port(gpio_t pin)
{
#if (CPU_FAM_NRF51)
    (void) pin;
    return NRF_GPIO;
#elif defined(CPU_MODEL_NRF52832XXAA)
    (void) pin;
    return NRF_P0;
#else
    return (pin & PORT_BIT) ? NRF_P1 : NRF_P0;
#endif
}

/**
 * @brief   Get a pin's offset
 */
static inline int pin_num(gpio_t pin)
{
#ifdef CPU_MODEL_NRF52840XXAA
    return (pin & PIN_MASK);
#else
    return (int)pin;
#endif
}

/**
 * @brief   Initialize the given pin as general purpose input or output
 *
 * When configured as output, the pin state after initialization is undefined.
 * The output pin's state **should** be untouched during the initialization.
 * This behavior can however **not be guaranteed** by every platform.
 *
 * @param[in] pin       pin to initialize
 * @param[in] mode      mode of the pin, see @c gpio_mode_t
 *
 * @return              0 on success
 * @return              -1 on error
 */
int gpio_init(gpio_t pin, 
              gpio_mode_t mode)
{
    if(pin == GPIO_UNDEF)
        return -1;

    switch (mode) {
        case GPIO_IN:       /**< Configure as input without pull resistor */
            port(pin)->PIN_CNF[pin] = mode;             
            break;
        case GPIO_IN_PD:    /**< Configure as input with pull-down resistor */
            port(pin)->PIN_CNF[pin] = mode;   
            break;
        case GPIO_IN_PU:    /**< Configure as input with pull-up resistor */
            port(pin)->PIN_CNF[pin] = mode;   
            break;
        case GPIO_OUT:      /**< Configure as output in push-pull mode */
            port(pin)->PIN_CNF[pin] = mode;   
            break;
        case GPIO_OD:       /**< Configure as output in open-drain mode without pull resistor */
            port(pin)->PIN_CNF[pin] = mode;   
            break;
        case GPIO_OD_PU:    /**< Configure as output in open-drain mode with pull resistor enabled */
            port(pin)->PIN_CNF[pin] = mode;   
            break;
        case GPIO_AIN:      /**< Configure as analog input */
            port(pin)->PIN_CNF[pin] = mode;
            break;    
        default:
            return -1;

    // switch (mode) {
    //     case GPIO_IN:       /**< Configure as input without pull resistor */
    //         port(pin)->PIN_CNF[pin] = ( (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)|
    //                                     (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos));             
    //         break;
    //     case GPIO_IN_PD:    /**< Configure as input with pull-down resistor */
    //         port(pin)->PIN_CNF[pin] = ( (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)|
    //                                     (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)); 
    //         break;
    //     case GPIO_IN_PU:    /**< Configure as input with pull-up resistor */
    //         port(pin)->PIN_CNF[pin] = ( (GPIO_PIN_CNF_DIR_Input   << GPIO_PIN_CNF_DIR_Pos)  |
    //                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)); 
    //         break;
    //     case GPIO_OUT:      /**< Configure as output in push-pull mode */
    //         port(pin)->PIN_CNF[pin] = (GPIO_PIN_CNF_DIR_Output    << GPIO_PIN_CNF_DIR_Pos); 
    //         break;
    //     case GPIO_OD:       /**< Configure as output in open-drain mode without pull resistor */
    //         port(pin)->PIN_CNF[pin] = ( (GPIO_PIN_CNF_DIR_Output  << GPIO_PIN_CNF_DIR_Pos)  |
    //                                     (GPIO_PIN_CNF_DRIVE_S0D1  << GPIO_PIN_CNF_DRIVE_Pos) ); 
    //         break;
    //     case GPIO_OD_PU:    /**< Configure as output in open-drain mode with pull resistor enabled */
    //         port(pin)->PIN_CNF[pin] = ( (GPIO_PIN_CNF_DIR_Output  << GPIO_PIN_CNF_DIR_Pos)  |
    //                                     (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
    //                                     (GPIO_PIN_CNF_DRIVE_S0D1  << GPIO_PIN_CNF_DRIVE_Pos) ); 
    //         break;
    //     case GPIO_AIN:      /**< Configure as analog input */
    //         //
    //         // port(pin)->PIN_CNF[pin] = ( ( << GPIO_PIN_CNF_DIR_Pos) |
    //         //                             ( << GPIO_PIN_CNF_INPUT_Pos) |
    //         //                             ( << GPIO_PIN_CNF_PULL_Pos) |
    //         //                             ( << GPIO_PIN_CNF_DRIVE_Pos) |
    //         //                             ( << GPIO_PIN_CNF_SENSE_Pos)); 
    //         break;    
    //     default:
    //         return -1;
    }

    return 0;
}

/**
 * @brief   Initialize a GPIO pin for external interrupt usage
 *
 * The registered callback function will be called in interrupt context every
 * time the defined flank(s) are detected.
 *
 * The interrupt is activated automatically after the initialization.
 *
 * @param[in] pin       pin to initialize
 * @param[in] mode      mode of the pin, see @c gpio_mode_t
 * @param[in] flank     define the active flank(s)
 * @param[in] cb        callback that is called from interrupt context
 * @param[in] arg       optional argument passed to the callback
 *
 * @return              0 on success
 * @return              -1 on error
 */
int gpio_init_int(gpio_t pin,
                  gpio_mode_t mode, 
                  gpio_flank_t flank,
                  gpio_cb_t cb, 
                  void *arg)
{
    if(pin == GPIO_UNDEF)
        return -1;

    /* Disable external interrupt in case one is active */
    NRF_GPIOTE->INTENSET &= ~(GPIOTE_INTENSET_IN0_Msk);

    /* Save callback */
    exti_chan.cb = cb;
    exti_chan.arg = arg;

    /* Configure pin as input */
    gpio_init(pin, mode);

    /* Set interrupt priority and enable global GPIOTE interrupt */
    NVIC_EnableIRQ(GPIOTE_IRQn);

    /* Configure the GPIOTE channel: set even mode, pin and active flank */
    NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event        |
                             (pin << GPIOTE_CONFIG_PSEL_Pos) |
#ifdef CPU_MODEL_NRF52840XXAA
                             ((pin & PORT_BIT) << 8)         |
#endif
                             (flank << GPIOTE_CONFIG_POLARITY_Pos));

    /* Enable external interrupt */
    NRF_GPIOTE->INTENSET |= GPIOTE_INTENSET_IN0_Msk;
    return 0;
}

/**
 * @brief   Enable pin interrupt if configured as interrupt source
 *
 * @param[in] pin       the pin to enable the interrupt for
 */
void gpio_irq_enable(gpio_t pin)
{
    (void) pin;
    NRF_GPIOTE->INTENSET |= GPIOTE_INTENSET_IN0_Msk;
}

/**
 * @brief   Disable the pin interrupt if configured as interrupt source
 *
 * @param[in] pin       the pin to disable the interrupt for
 */
void gpio_irq_disable(gpio_t pin)
{
    (void) pin;
    NRF_GPIOTE->INTENCLR |= GPIOTE_INTENSET_IN0_Msk;
}

/**
 * @brief   Get the current value of the given pin
 *
 * @param[in] pin       the pin to read
 *
 * @return              0 when pin is LOW
 * @return              >0 for HIGH
 */
int gpio_read(gpio_t pin)
{
    if (port(pin)->DIR & (1 << pin)) {
        return (port(pin)->OUT & (1 << pin)) ? 1 : 0;
    }
    else {
        return (port(pin)->IN & (1 << pin)) ? 1 : 0;
    }
}

/**
 * @brief   Set the given pin to HIGH
 *
 * @param[in] pin       the pin to set
 */
void gpio_set(gpio_t pin)
{
    port(pin)->OUTSET = (1 << pin);
}

/**
 * @brief   Set the given pin to LOW
 *
 * @param[in] pin       the pin to clear
 */
void gpio_clear(gpio_t pin)
{
    port(pin)->OUTCLR = (1 << pin);
}

/**
 * @brief   Toggle the value of the given pin
 *
 * @param[in] pin       the pin to toggle
 */
void gpio_toggle(gpio_t pin)
{
    port(pin)->OUT ^= (1 << pin);
}

/**
 * @brief   Set the given pin to the given value
 *
 * @param[in] pin       the pin to set
 * @param[in] value     value to set the pin to, 0 for LOW, HIGH otherwise
 */
void gpio_write(gpio_t pin, int value)
{
    if (value) {
        port(pin)->OUTSET = (1 << pin);
    } else {
        port(pin)->OUTCLR = (1 << pin);
    }
}

/**
 * @brief   Interrupt service routine GPIOTE
 */
void isr_gpiote(void)
{
    if (NRF_GPIOTE->EVENTS_IN[0] == 1) {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        exti_chan.cb(exti_chan.arg);
    }
    cortexm_isr_end();
}
