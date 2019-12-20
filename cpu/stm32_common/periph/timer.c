/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_timer
 * @{
 *
 * @file
 * @brief       Low-level timer driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "periph/timer.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

/**
 * @brief   Interrupt context for each configured timer
 */
static timer_isr_ctx_t isr_ctx[TIMER_NUMOF];

/**
 * @brief   Get the timer device
 */
static inline TIM_TypeDef *dev(tim_t tim)
{
    return timer_config[tim].dev;
}

int timer_init(tim_t tim, unsigned long freq, timer_cb_t cb, void *arg)
{
    /* check if device is valid */
    if (tim >= TIMER_NUMOF) {
        return -1;
    }

    /* remember the interrupt context */
    isr_ctx[tim].cb = cb;
    isr_ctx[tim].arg = arg;

    /* enable the peripheral clock */
    periph_clk_en(timer_config[tim].bus, timer_config[tim].rcc_mask);

    /* configure the timer as upcounter in continuous mode */
    dev(tim)->CR1  = 0;
    dev(tim)->CR2  = 0;
    dev(tim)->ARR  = timer_config[tim].max;

    /* set prescaler */
    dev(tim)->PSC = ((periph_timer_clk(timer_config[tim].bus) / freq) - 1);
    /* generate an update event to apply our configuration */
    dev(tim)->EGR = TIM_EGR_UG;

    /* enable the timer's interrupt */
    NVIC_EnableIRQ(timer_config[tim].irqn);
    /* reset the counter and start the timer */
    timer_start(tim);

    return 0;
}

int timer_init_periodic(tim_t tim, uint32_t period, timer_cb_t cb, void *arg, bool signal) {
    /* check if device is valid */
    if (tim >= TIMER_NUMOF) {
        return -1;
    }
    
    if (!period) {
        return -1;
    }
    
    uint32_t freq_uhz = periph_timer_clk(timer_config[tim].bus) / 1000000;
    uint32_t prescaler = ((10 * freq_uhz * period) + 5) / (10 * timer_config[tim].max);
    
    if (prescaler > timer_config[tim].max) {
        return -1;
    }
    
    /* remember the interrupt context */
    isr_ctx[tim].cb = cb;
    isr_ctx[tim].arg = arg;
    
    /* enable the peripheral clock */
    periph_clk_en(timer_config[tim].bus, timer_config[tim].rcc_mask);
    
    if (cb != NULL) {
        isr_ctx[tim].cb = cb;
        isr_ctx[tim].arg = arg;
        
        /* enable the timer's update interrupt */
        NVIC_EnableIRQ(timer_config[tim].irqn);
        dev(tim)->DIER |= TIM_DIER_UIE;
    } else {
        /* disable update interrupt */
        dev(tim)->DIER &= ~TIM_DIER_UIE;
    }
    
    /* configure the timer as upcounter in continuous mode */
    dev(tim)->CR1  = 0;
    dev(tim)->CR2  = 0;
    
    /* set prescaler */
    dev(tim)->PSC = prescaler;
    
    /* set reload value */
    dev(tim)->ARR = ((10 * period * freq_uhz) + 5 ) / (20 * (prescaler + 1)) - 1;
    
    DEBUG("Freq: %" PRIu32 ", PSC: %" PRIu32 ", ARR: %" PRIu32 "\n", periph_timer_clk(timer_config[tim].bus), (uint32_t)dev(tim)->PSC, dev(tim)->ARR);

    /* generate an update event to apply our configuration */
    dev(tim)->EGR = TIM_EGR_UG;
    
    dev(tim)->CR2 &= ~TIM_CR2_MMS;
    if (signal) {
        dev(tim)->CR2 |= TIM_CR2_MMS_1;
    }
    
    return 0;
}

int timer_set_freq(tim_t tim, unsigned long freq)
{
    /* check if given timer exists */
    if (tim >= TIMER_NUMOF) {
        return -1;
    }

    /* enable peripheral clock */
	periph_clk_en(timer_config[tim].bus, timer_config[tim].rcc_mask);
	/* change prescaler */
    dev(tim)->PSC = (cpu_status.clock.coreclock / freq) - 1;
    /* trigger update event to make pre-scaler value effective */
    dev(tim)->EGR = TIM_EGR_UG;
    return 0;
}

unsigned long timer_get_freq(tim_t tim)
{
    /* check if given timer exists */
    if (tim >= TIMER_NUMOF) {
        return 0;
    }

    /* enable peripheral clock */
	periph_clk_en(timer_config[tim].bus, timer_config[tim].rcc_mask);
	/* change prescaler */
	
	return cpu_status.clock.coreclock/((dev(tim)->PSC) + 1);
}

int timer_set(tim_t tim, int channel, unsigned int timeout)
{
    int now = timer_read(tim);
    return timer_set_absolute(tim, channel, now + timeout);
}

int timer_set_absolute(tim_t tim, int channel, unsigned int value)
{
    if (channel >= (int)TIMER_CHAN) {
        return -1;
    }

    dev(tim)->CCR[channel] = (value & timer_config[tim].max);
    dev(tim)->SR &= ~(TIM_SR_CC1IF << channel);
    dev(tim)->DIER |= (TIM_DIER_CC1IE << channel);

    return 0;
}

int timer_clear(tim_t tim, int channel)
{
    if (channel >= (int)TIMER_CHAN) {
        return -1;
    }

    dev(tim)->DIER &= ~(TIM_DIER_CC1IE << channel);
    return 0;
}

unsigned int timer_read(tim_t tim)
{
    return (unsigned int)dev(tim)->CNT;
}

void timer_start(tim_t tim)
{
    dev(tim)->CR1 |= TIM_CR1_CEN;
}

void timer_stop(tim_t tim)
{
    dev(tim)->DIER &= ~TIM_DIER_UIE;
    dev(tim)->CR1 &= ~(TIM_CR1_CEN);
}

static inline void irq_handler(tim_t tim)
{
    uint32_t status = (dev(tim)->SR & dev(tim)->DIER);

    for (unsigned int i = 0; i < TIMER_CHAN; i++) {
        /* timer_init triggers Capture/Compare Event */
        if (status & (TIM_SR_CC1IF << i)) {
            dev(tim)->DIER &= ~(TIM_DIER_CC1IE << i);
            isr_ctx[tim].cb(isr_ctx[tim].arg, i);
        }
        
        /* timer_init_periodic triggers Update Event */
        if (status & (TIM_SR_UIF << i)) {
            dev(tim)->SR &= ~TIM_SR_UIF;
            isr_ctx[tim].cb(isr_ctx[tim].arg, i);
        }
    }
    cortexm_isr_end();
}

#ifdef TIMER_0_ISR
void TIMER_0_ISR(void)
{
    irq_handler(0);
}
#endif

#ifdef TIMER_1_ISR
void TIMER_1_ISR(void)
{
    irq_handler(1);
}
#endif

#ifdef TIMER_2_ISR
void TIMER_2_ISR(void)
{
    irq_handler(2);
}
#endif

#ifdef TIMER_3_ISR
void TIMER_3_ISR(void)
{
    irq_handler(3);
}
#endif

#ifdef TIMER_4_ISR
void TIMER_4_ISR(void)
{
    irq_handler(4);
}
#endif
