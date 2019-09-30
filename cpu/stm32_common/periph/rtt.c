/*
 * Copyright (C) 2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32_common
 * @ingroup     drivers_periph_rtt
 * @{
 *
 * @file
 * @brief       RTT implementation using LPTIM1
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "irq.h"
#include "periph/rtt.h"
#include "stmclk.h"

/* temporary as RTT based on LPTIM1 is not stable yet */
#undef LPTIM1

/* this driver is only valid for STM CPUs that provide LPTIMERs */
#if defined(LPTIM1)

#error "Shitty code"

/* figure out the used pre-scaler */
#if (RTT_FREQUENCY == 32768)
#define PRE                 (0)
#elif (RTT_FREQUENCY == 16384)
#define PRE                 (LPTIM_CFGR_PRESC_0)
#elif (RTT_FREQUENCY == 8192)
#define PRE                 (LPTIM_CFGR_PRESC_1)
#elif (RTT_FREQUENCY == 4096)
#define PRE                 (LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_PRESC_0)
#elif (RTT_FREQUENCY == 2048)
#define PRE                 (LPTIM_CFGR_PRESC_2)
#elif (RTT_FREQUENCY == 1024)
#define PRE                 (LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_0)
#elif (RTT_FREQUENCY == 512)
#define PRE                 (LPTIM_CFGR_PRESC_2 | LPTIM_CFGR_PRESC_1)
#elif (RTT_FREQUENCY == 256)
#define PRE                 (LPTIM_CFGR_PRESC)
#else
#error "RTT config: RTT_FREQUENCY not configured or invalid for your board"
#endif


#if !defined(CPU_FAM_STM32F4)
#define CLOCK_SRC_REG       RCC->CCIPR
#define CLOCK_SRC_MASK      RCC_CCIPR_LPTIM1SEL
#if CLOCK_LSE
#define CLOCK_SRC_CFG       (RCC_CCIPR_LPTIM1SEL_1 | RCC_CCIPR_LPTIM1SEL_0)
#else
#define CLOCK_SRC_CFG       (RCC_CCIPR_LPTIM1SEL_0)
#endif
#else
#define CLOCK_SRC_REG       RCC->DCKCFGR2
#define CLOCK_SRC_MASK      RCC_DCKCFGR2_LPTIM1SEL
#if CLOCK_LSE
#define CLOCK_SRC_CFG       (RCC_DCKCFGR2_LPTIM1SEL_1 | RCC_DCKCFGR2_LPTIM1SEL_0)
#else
#define CLOCK_SRC_CFG       (RCC_DCKCFGR2_LPTIM1SEL_0)
#endif
#endif


/* allocate memory for overflow and alarm callbacks + args */
static rtt_cb_t ovf_cb = NULL;
static void *ovf_arg;
static rtt_cb_t to_cb = NULL;
static void *to_arg;

void rtt_init(void)
{
    stmclk_enable_lfclk();
    /* power on the selected LPTIMER */
    rtt_poweron();

    /* stop the timer and reset configuration */
    LPTIM1->CR = 0;

    /* select low speed clock (LSI or LSE) */
    CLOCK_SRC_REG &= ~(CLOCK_SRC_MASK);
    CLOCK_SRC_REG |= CLOCK_SRC_CFG;

    /* set configuration: prescale factor and external clock (LSI or LSE) */
    LPTIM1->CFGR = PRE;
    /* enable overflow and compare interrupts */
    LPTIM1->IER = (LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE);
    NVIC_EnableIRQ(LPTIM1_IRQn);
    /* enable timer */
    LPTIM1->CR = LPTIM_CR_ENABLE;
    /* set auto-reload value (timer needs to be enabled for this) */
    LPTIM1->ICR = LPTIM_ICR_ARROKCF;
    LPTIM1->ARR = RTT_MAX_VALUE;
    while (!(LPTIM1->ISR & LPTIM_ISR_ARROK)) {}
    /* start the timer */
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;
}

uint32_t rtt_get_counter(void)
{
    uint32_t cnt;
    do {
        cnt = LPTIM1->CNT;
    } while (cnt != LPTIM1->CNT);
    return cnt;
}

void rtt_set_overflow_cb(rtt_cb_t cb, void *arg)
{
    assert(cb);

    unsigned is = irq_disable();
    ovf_cb  = cb;
    ovf_arg = arg;
    irq_restore(is);
}

void rtt_clear_overflow_cb(void)
{
    ovf_cb = NULL;
}

void rtt_set_alarm(uint32_t alarm, rtt_cb_t cb, void *arg)
{
    assert(cb && !(alarm & ~RTT_MAX_VALUE));

    unsigned is = irq_disable();
    LPTIM1->ICR = LPTIM_ICR_CMPOKCF;
    to_cb  = cb;
    to_arg = arg;
    LPTIM1->CMP = (uint16_t)alarm;
    while (!(LPTIM1->ISR & LPTIM_ISR_CMPOK)) {}
    irq_restore(is);
}

void rtt_clear_alarm(void)
{
    to_cb = NULL;
}

void rtt_poweron(void)
{
#ifdef RCC_APB1ENR1_LPTIM1EN
    periph_clk_en(APB1, RCC_APB1ENR1_LPTIM1EN);
#else
    periph_clk_en(APB1, RCC_APB1ENR_LPTIM1EN);
#endif
}

void rtt_poweroff(void)
{
#ifdef RCC_APB1ENR1_LPTIM1EN
    periph_clk_dis(APB1, RCC_APB1ENR1_LPTIM1EN);
#else
    periph_clk_dis(APB1, RCC_APB1ENR_LPTIM1EN);
#endif
}

void isr_lptim1(void)
{
    if (LPTIM1->ISR & LPTIM_ISR_CMPM) {
        if (to_cb) {
            /* 'consume' the callback (as it might be set again in the cb) */
            rtt_cb_t tmp = to_cb;
            to_cb = NULL;
            tmp(to_arg);
        }
    }
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        if (ovf_cb) {
            ovf_cb(ovf_arg);
        }
    }
    LPTIM1->ICR = (LPTIM_ICR_ARRMCF | LPTIM_ICR_CMPMCF);

    cortexm_isr_end();
}

/* if no LPTIM1 is available, we use RTT-on-RTC emulation
 * only STM32L1 Cat. 2 and newer MCUs are supported
 * not compatible with regular RTC due to clock settings */
 
/* F1 doesn't have RTC_SSR, L0, L4, G0, F7, H7 have LPTIMs */
#elif defined(CPU_FAM_STM32L0) || defined(CPU_FAM_STM32L1) || \
      defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32F2) || \
      defined(CPU_FAM_STM32F3) || defined(CPU_FAM_STM32F4)

#if defined(CPU_FAM_STM32L1) || defined(CPU_FAM_STM32L0)
#define EN_REG              (RCC->CSR)
#define EN_BIT              (RCC_CSR_RTCEN)
#define RST_BIT             (RCC_CSR_RTCRST)
#define CLKSEL_MASK         (RCC_CSR_RTCSEL)
#define CLKSEL_LSE          (RCC_CSR_RTCSEL_LSE)
#define CLKSEL_LSI          (RCC_CSR_RTCSEL_LSI)
#else
#define EN_REG              (RCC->BDCR)
#define EN_BIT              (RCC_BDCR_RTCEN)
#define CLKSEL_MASK         (RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCSEL_1)
#define CLKSEL_LSE          (RCC_BDCR_RTCSEL_0)
#define CLKSEL_LSI          (RCC_BDCR_RTCSEL_1)
#endif

/* interrupt line name mapping */
#if defined(CPU_FAM_STM32F0) || defined(CPU_FAM_STM32L0)
#define IRQN                (RTC_IRQn)
#define IRQNWU              (RTC_IRQn)
#define ISR_NAME            isr_rtc
#else
#define IRQN                (RTC_Alarm_IRQn)
#define IRQNWU              (RTC_WKUP_IRQn)
#define ISR_NAME            isr_rtc_alarm
#endif

/* write protection values */
#define WPK1                (0xCA)
#define WPK2                (0x53)

#if defined(CPU_FAM_STM32L0)
#define EXTI_IMR_BIT        (EXTI_IMR_IM17)
#define EXTI_IMRWU_BIT      (EXTI_IMR_IM20)
#else
#define EXTI_IMR_BIT        (EXTI_IMR_MR17)
#define EXTI_IMRWU_BIT      (EXTI_IMR_MR20)
#endif
#define EXTI_FTSR_BIT       (EXTI_FTSR_TR17)
#define EXTI_RTSR_BIT       (EXTI_RTSR_TR17)
#define EXTI_PR_BIT         (EXTI_PR_PR17)

/* figure out RTT clock */
#if defined(RTT_FREQUENCY)
    #if CLOCK_LSE
        #define PRE_ASYNC           ((uint32_t)(32768/RTT_FREQUENCY) - 1) /* 31 */
    #elif CLOCK_LSI
        #define PRE_ASYNC           ((uint32_t)(CLOCK_LSI/RTT_FREQUENCY))
    #else
        #error "rtt (on rtc): no LSI or LSE clock defined"
    #endif
#else
#error "rtt (on rtc): RTT_FREQUENCY undefined"
#endif

#define PRE_SYNC    (0x7FFFul) /* full 15 bits */ /* 32767 */

rtt_cb_t cb_a;
void *arg_a;

static inline void rtc_unlock(void)
{
    /* unlock RTC */
    RTC->WPR = WPK1;
    RTC->WPR = WPK2;
}

static inline void rtc_lock(void)
{
    /* lock RTC device */
    RTC->WPR = 0xff;
}

void rtt_init(void) {
    /* enable low frequency clock */
    stmclk_enable_lfclk();

    /* select input clock and enable the RTC */
    stmclk_dbp_unlock();
    
    EN_REG &= ~(CLKSEL_MASK);
#if CLOCK_LSE
    EN_REG |= (CLKSEL_LSE | EN_BIT);
#else
    EN_REG |= (CLKSEL_LSI | EN_BIT);
#endif

    rtc_unlock();
    /* enter RTC init mode */
    RTC->ISR |= RTC_ISR_INIT;
    while (!(RTC->ISR & RTC_ISR_INITF)) {}
    /* reset configuration */
    RTC->CR = 0;
    RTC->ISR = RTC_ISR_INIT;
    /* configure prescaler (RTC PRER) */
    RTC->PRER = PRE_SYNC | (PRE_ASYNC << 16);
    /* Set 24-h clock */
    RTC->CR &= ~RTC_CR_FMT;
    /* Timestamps disabled */
    RTC->CR &= ~RTC_CR_TSE;
    
    /* exit RTC init mode */
    RTC->ISR &= ~RTC_ISR_INIT;
    while (RTC->ISR & RTC_ISR_INITF) {}
    
    rtc_lock();

    /* configure the EXTI channel, as RTC interrupts are routed through it.
     * Needs to be configured to trigger on rising edges. */
    EXTI->FTSR &= ~(EXTI_FTSR_BIT);
    EXTI->RTSR |= EXTI_RTSR_BIT;
    EXTI->IMR  |= EXTI_IMR_BIT;
    EXTI->PR   |= EXTI_PR_BIT;
    /* enable global RTC interrupt */
    NVIC_EnableIRQ(IRQN);
}

uint32_t rtt_get_counter(void)
{
    /* clear RSF bit */
    RTC->ISR &= ~RTC_ISR_RSF;
    
    /* wait for RSF to be set by hardware */
    while (!(RTC->ISR & RTC_ISR_RSF)) {}

    uint32_t rtc_ssr_counter = RTC->SSR;
    
    RTC->DR;
    
    /* it's a downcounting timer */
    return (RTT_MAX_VALUE - rtc_ssr_counter);
}

void rtt_set_overflow_cb(rtt_cb_t cb, void *arg)
{
    (void)cb;
    (void)arg;
    /* not implemented yet */
}

void rtt_clear_overflow_cb(void)
{
    /* not implemented yet */
}

void rtt_set_alarm(uint32_t alarm, rtt_cb_t cb, void *arg)
{
    rtc_unlock();
    
    RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
    while (!(RTC->ISR & RTC_ISR_ALRAWF)) {}
    
    /* seconds, minutes, hours and date doesn't matter */
    RTC->ALRMAR |= (RTC_ALRMAR_MSK1 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK4);
    
    /* it's a downcounting timer */
    alarm = RTT_MAX_VALUE - alarm;
    
    /* compare all 15 bits */
    RTC->ALRMASSR = (alarm & RTT_MAX_VALUE) | RTC_ALRMASSR_MASKSS;
    
    /* Enable Alarm */
    RTC->CR |= RTC_CR_ALRAE;
    RTC->CR |= RTC_CR_ALRAIE;
    RTC->ISR &= ~(RTC_ISR_ALRAF);

    cb_a = cb;
    arg_a = arg;
    
    rtc_lock();
}

void rtt_clear_alarm(void)
{
    rtc_unlock();
    RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
    while (!(RTC->ISR & RTC_ISR_ALRAWF)) {}
    
    cb_a = NULL;
    arg_a = NULL;

    rtc_lock();
}

void rtt_poweron(void)
{
    stmclk_dbp_unlock();
    EN_REG |= EN_BIT;
}

void rtt_poweroff(void)
{
    stmclk_dbp_unlock();
    EN_REG &= ~EN_BIT;
    stmclk_dbp_lock();
}

void ISR_NAME(void)
{
    if (RTC->ISR & RTC_ISR_ALRAF) {
        if (cb_a != NULL) {
            cb_a(arg_a);
        }
        RTC->ISR &= ~RTC_ISR_ALRAF;
        EXTI->PR |= EXTI_PR_BIT;
    }
    
    cortexm_isr_end();
}

#endif
