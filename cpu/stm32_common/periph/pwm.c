/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *               2015 Engineering-Spirit
 *               2016 OTA keys S.A.
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_pwm
 * @{
 *
 * @file
 * @brief       Low-level PWM driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Fabian Nack <nack@inf.fu-berlin.de>
 * @author      Nick v. IJzendoorn <nijzendoorn@engineering-spirit.nl>
 * @author      Aurelien Gonce <aurelien.gonce@altran.fr>
 *
 * @}
 */

#include "cpu.h"
#include "assert.h"
#include "periph/pwm.h"
#include "periph/gpio.h"

#define CCMR_LEFT           (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | \
                             TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)
#define CCMR_RIGHT          (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | \
                             TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_0 | \
                             TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

volatile uint32_t pwm_pulses_counter[PWM_NUMOF] = { 0 };

static inline TIM_TypeDef *dev(pwm_t pwm)
{
    return pwm_config[pwm].dev;
}

uint32_t pwm_init(pwm_t pwm, pwm_mode_t mode, uint32_t freq, uint16_t res)
{
    uint32_t timer_clk = periph_timer_clk(pwm_config[pwm].bus);

    /* verify parameters */
    assert((pwm < PWM_NUMOF) && ((freq * res) <= timer_clk));

    /* power on the used timer */
    periph_clk_en(pwm_config[pwm].bus, pwm_config[pwm].rcc_mask);

    /* configure the PWM frequency and resolution by setting the auto-reload
     * and prescaler registers */
    dev(pwm)->PSC = (timer_clk / (res * freq)) - 1;
    dev(pwm)->ARR = res - 1;

    /* set PWM mode */
    switch (mode) {
        case PWM_LEFT:
            dev(pwm)->CCMR1 = CCMR_LEFT;
            dev(pwm)->CCMR2 = CCMR_LEFT;
            dev(pwm)->CR1 &= ~(TIM_CR1_CMS_0 | TIM_CR1_CMS_1);
            break;
        case PWM_RIGHT:
            dev(pwm)->CCMR1 = CCMR_RIGHT;
            dev(pwm)->CCMR2 = CCMR_RIGHT;
            dev(pwm)->CR1 &= ~(TIM_CR1_CMS_0 | TIM_CR1_CMS_1);
            break;
        case PWM_CENTER:
            dev(pwm)->CCMR1 = 0;
            dev(pwm)->CCMR2 = 0;
            dev(pwm)->CR1 |= (TIM_CR1_CMS_0 | TIM_CR1_CMS_1);
            break;
    }

    /* enable PWM outputs and start PWM generation */
#ifdef TIM_BDTR_MOE
    dev(pwm)->BDTR = TIM_BDTR_MOE;
#endif
    dev(pwm)->CCER = (TIM_CCER_CC1E | TIM_CCER_CC2E |
                      TIM_CCER_CC3E | TIM_CCER_CC4E);

    /* return the actual used PWM frequency */
    return (timer_clk / (res * (dev(pwm)->PSC + 1)));
}

uint8_t pwm_channels(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    
    (void) pwm;
    
    unsigned i = 0;
    while (i < TIMER_CHAN) {
        i++;
    }
    return (uint8_t)i;
}

void pwm_set(pwm_t pwm, uint8_t channel, uint16_t value)
{
    assert((pwm < PWM_NUMOF) &&
           (channel < TIMER_CHAN));

    if (pwm_config[pwm].chan[channel].pin != GPIO_UNDEF) {
        /* norm value to maximum possible value */
        if (value > dev(pwm)->ARR) {
            value = (uint16_t)dev(pwm)->ARR;
        }
        /* set new value */
        dev(pwm)->CCR[pwm_config[pwm].chan[channel].cc_chan] = value;
    }
}

void pwm_start(pwm_t pwm, uint8_t channel)
{
    assert(pwm < PWM_NUMOF);
    
    uint32_t pin = pwm_config[pwm].chan[channel].pin;
    
    if (pin == GPIO_UNDEF) {
        return;
    }
    
    /* reimplementing gpio_init_af here to split computations and register writes */
    GPIO_TypeDef *port = (GPIO_TypeDef *)(pin & ~(0x0f));
    uint32_t pin_num = (pin & 0x0f);

    uint32_t irqs = irq_disable();

    /* set pin to AF mode */
    uint32_t moder;
	moder = port->MODER;
    moder &= ~(3 << (2 * pin_num));
    moder |= (2 << (2 * pin_num));
    
    /* set selected function */
    uint32_t afr;
	afr = port->AFR[(pin_num > 7) ? 1 : 0];
    afr &= ~(0xf << ((pin_num & 0x07) * 4));
    afr |= (pwm_config[pwm].af << ((pin_num & 0x07) * 4));
    uint32_t afr_num = (pin_num > 7) ? 1 : 0;
    
    /* enable PWM */
    dev(pwm)->CR1 |= TIM_CR1_CEN;
    
    /* delay needed to eliminate small glitch, present at least on STM32L1 */
    __asm("nop; nop; nop; nop; nop;");

    /* if pin was configured before timer started, glitch is bigger */
    port->AFR[afr_num] = afr;
    port->MODER = moder;
    
    /* restore interrupts */
    irq_restore(irqs);
}

void pwm_pulses(pwm_t pwm, uint8_t channel, uint16_t pulses) {
    assert(pwm < PWM_NUMOF);

    pwm_pulses_counter[pwm] = (pulses | (channel << 16)) + 1;

    /* configure update event interrupt */
    dev(pwm)->DIER |= (TIM_DIER_CC1IE << pwm_config[pwm].chan[channel].cc_chan);
    NVIC_EnableIRQ(pwm_config[pwm].irqn);

    pwm_start(pwm, channel);
    
    /* blocking function */
    while (dev(pwm)->CR1 & TIM_CR1_CEN) {};
    
    dev(pwm)->DIER &= ~(TIM_DIER_CC1IE << pwm_config[pwm].chan[channel].cc_chan);
}

void pwm_stop(pwm_t pwm, uint8_t channel)
{
    assert(pwm < PWM_NUMOF);
    
    /* disable corresponding pin */
    gpio_init(pwm_config[pwm].chan[channel].pin, GPIO_OUT);
    gpio_clear(pwm_config[pwm].chan[channel].pin);
}

void pwm_poweron(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    periph_clk_en(pwm_config[pwm].bus, pwm_config[pwm].rcc_mask);
    dev(pwm)->CR1 |= TIM_CR1_CEN;
}

void pwm_poweroff(pwm_t pwm)
{
    assert(pwm < PWM_NUMOF);
    dev(pwm)->CR1 &= ~TIM_CR1_CEN;
    periph_clk_dis(pwm_config[pwm].bus, pwm_config[pwm].rcc_mask);
}

static inline void irq_handler(pwm_t pwm) {
    uint32_t pulses = pwm_pulses_counter[pwm];
    uint16_t channel = pulses >> 16;
    pulses &= 0xFFFF;

    if (dev(pwm)->SR & (TIM_SR_CC1IF << pwm_config[pwm].chan[channel].cc_chan)) {
        /* reset capture/compare interrupt flag */
        dev(pwm)->SR &= ~(TIM_SR_CC1IF << pwm_config[pwm].chan[channel].cc_chan);
        
        if (pulses) {
            pulses--;
            
            if (!pulses) {
                dev(pwm)->CR1 &= ~TIM_CR1_CEN;
            }
            pwm_pulses_counter[pwm] = pulses | (channel << 16);
        }
    }
    
    cortexm_isr_end();
}

#ifdef TIM_0_ISR
void TIM_0_ISR(void)
{
    irq_handler(PWM_DEV(0));
}
#endif /* TIM_0_ISR */

#ifdef TIM_1_ISR
void TIM_1_ISR(void)
{
    irq_handler(PWM_DEV(1));
}
#endif /* TIM_1_ISR */

#ifdef TIM_2_ISR
void TIM_2_ISR(void)
{
    irq_handler(PWM_DEV(2));
}
#endif /* TIM_2_ISR */

#ifdef TIM_3_ISR
void TIM_3_ISR(void)
{
    irq_handler(PWM_DEV(3));
}
#endif /* TIM_3_ISR */

#ifdef TIM_4_ISR
void TIM_4_ISR(void)
{
    irq_handler(PWM_DEV(4));
}
#endif /* TIM_4_ISR */

#ifdef TIM_5_ISR
void TIM_5_ISR(void)
{
    irq_handler(PWM_DEV(5));
}
#endif /* TIM_5_ISR */

#ifdef TIM_6_ISR
void TIM_6_ISR(void)
{
    irq_handler(PWM_DEV(6));
}
#endif /* TIM_6_ISR */

#ifdef TIM_7_ISR
void TIM_7_ISR(void)
{
    irq_handler(PWM_DEV(7));
}
#endif /* TIM_7_ISR */

#ifdef TIM_8_ISR
void TIM_8_ISR(void)
{
    irq_handler(PWM_DEV(8));
}
#endif /* TIM_8_ISR */

#ifdef TIM_9_ISR
void TIM_9_ISR(void)
{
    irq_handler(PWM_DEV(9));
}
#endif /* TIM_9_ISR */
