/*
 * Copyright (C) 2015 Lari Lehtomäki
 *               2016 Laksh Bhatia
 *               2016-2017 OTA keys S.A.
 *               2017 Freie Universität Berlin
 *               2017 Unwired Devices LLC [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32f1
 * @{
 * @file
 * @brief       Low-level RTC driver implementation
 *
 * @author      Mikhail Perkov
 * @}
 */

#include <time.h>
#include "cpu.h"
#include "stmclk.h"
#include "periph/rtc.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#ifndef PERIPH_RTC
#define PERIPH_RTC

/* interrupt line name mapping */

#define ISR_NAME            isr_rtc
// #define IRQN                (RTC_Alarm_IRQn)
// #define IRQNWU              (RTC_WKUP_IRQn)
// #define ISR_NAME            isr_rtc_alarm

#define RTC_FLAG_RTOFF       ((uint16_t)0x0020)  /**< RTC Operation OFF flag */
#define RTC_FLAG_RSF         ((uint16_t)0x0008)  /**< Registers Synchronized flag */
#define RTC_FLAG_OW          ((uint16_t)0x0004)  /**< Overflow flag */
#define RTC_FLAG_ALR         ((uint16_t)0x0002)  /**< Alarm flag */
#define RTC_FLAG_SEC         ((uint16_t)0x0001)  /**< Second flag */

/*
 * callback and argument for an active alarm
 */
static rtc_alarm_cb_t alarm_cb;
static void *alarm_arg;

static inline void _rtc_enter_config_mode(void);
static inline void _rtc_leave_config_mode(void);

static inline void _rtc_enter_config_mode(void)
{
    /* Loop until RTOFF flag is set */
    while (!(RTC->CRL & RTC_FLAG_RTOFF)) {}
    /* enter configuration mode */
    RTC->CRL |= RTC_CRL_CNF;
}

static inline void _rtc_leave_config_mode(void)
{
    /* leave configuration mode */
    RTC->CRL &= ~RTC_CRL_CNF;
    /* Loop until RTOFF flag is set */
    while (!(RTC->CRL & RTC_FLAG_RTOFF)) {}
}

int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg)
{
    time_t time_unix = mktime(time);
    _rtc_enter_config_mode();

    /* Set the alarm MSB word */
    RTC->ALRH = time_unix >> 16;
    /* Set the alarm LSB word */
    RTC->ALRL = (time_unix & 0xFFFF);
    /* Enable alarm interrupt */
    RTC->CRH |= RTC_CRH_ALRIE;

    _rtc_leave_config_mode();

    alarm_cb = cb;
    alarm_arg = arg;
    
    return 0;
}

int rtc_get_alarm(struct tm *time)
{
    /* wait for syncronization */
    while (!(RTC->CRL & RTC_FLAG_RSF)) {}

    time_t time_unix = (((uint32_t)RTC->ALRH << 16 ) | (uint32_t)(RTC->ALRL));
    
    *time = *gmtime(&time_unix);
    
    return 0;
}

void rtc_clear_alarm(void)
{
    _rtc_enter_config_mode();

    /* disable alarm interrupt */
    RTC->CRH &= ~RTC_CRH_ALRIE;

    _rtc_leave_config_mode();

    alarm_cb = NULL;
    alarm_arg = NULL;
}

int rtc_save_backup(uint32_t data, uint8_t reg_num) {    
    __IO uint32_t tmp = 0;
    
    if (reg_num < 5) {
        tmp = BKP_BASE + 0x04;
    } else {
        tmp = BKP_BASE + 0x40;
    }
    tmp += (reg_num * 8);

    /* Write the first half (16-bit) */
    *(__IO uint32_t *)tmp = (uint32_t)(data & 0xFFFF);

    /* Write the second half (16-bit) */
    *(__IO uint32_t *)(tmp + 4) = (uint32_t)(data >> 16);

    return 0;
}

uint32_t rtc_restore_backup(uint8_t reg_num) {
    __IO uint32_t tmp = 0;
    
    if (reg_num < 5) {
        tmp = BKP_BASE + 0x04;
    } else {
        tmp = BKP_BASE + 0x40;
    }
    tmp += (reg_num * 8);

    /* Read the specified register */
    return (*(__IO uint32_t *)tmp & 0xFFFF) | (*(__IO uint32_t *)(tmp + 4) << 16);
}


void rtc_init(void)
{       
    periph_clk_en(APB1, RCC_APB1ENR_BKPEN);  /* enable BKP, Clock */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);  /* Power domain enable */
    /* RTC clock source configuration */
    PWR->CR |= PWR_CR_DBP;                   /* Allow access to BKP Domain */
    RCC->BDCR |= RCC_BDCR_LSEON;             /* Enable LSE OSC */
    while(!(RCC->BDCR & RCC_BDCR_LSERDY)) {} /* Wait till LSE is ready */
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE;        /* Select the RTC Clock Source */
    RCC->BDCR |= RCC_BDCR_RTCEN;             /* enable RTC */
    
        /* configure interrupt */
    NVIC_SetPriority(RTC_IRQn, RTC_IRQ_PRIO);
    NVIC_EnableIRQ(RTC_IRQn);

    /* clear RSF flag */
    RTC->CRL &= ~(RTC_FLAG_RSF);
    
    _rtc_enter_config_mode();
       
    /* set prescaler */
    RTC->PRLH = ((RTC_PRESCALER >> 16) & 0x000F);
    RTC->PRLL = (RTC_PRESCALER & 0xFFFF);
    
    _rtc_leave_config_mode();  
}

int rtc_set_time(struct tm *time)
{ 
    time_t time_unix = mktime(time);
    _rtc_enter_config_mode();

    /* Set RTC counter MSB word */
    RTC->CNTH = time_unix >> 16;
    /* Set RTC counter LSB word */
    RTC->CNTL = (time_unix & 0xFFFF);

    _rtc_leave_config_mode();    
    
    return 0;
}

int rtc_get_time(struct tm *time)
{
        /* wait for syncronization */
    while (!(RTC->CRL & RTC_FLAG_RSF)) {}
    time_t time_unix = (((uint32_t)RTC->CNTH << 16 ) | (uint32_t)(RTC->CNTL));
       
    *time = *gmtime(&time_unix);
    
    return 0;
}

void rtc_poweron(void)
{
    RCC->BDCR |= RCC_BDCR_RTCEN;             /* enable RTC */
}

void rtc_poweroff(void)
{
    RCC->BDCR &= ~RCC_BDCR_RTCEN;            /* disable RTC */
}

void ISR_NAME(void)
{
    if (RTC->CRL & RTC_CRL_ALRF) {
        RTC->CRL &= ~(RTC_CRL_ALRF);
        alarm_cb(alarm_arg);
    }
    
    cortexm_isr_end();
}

#endif /* RTC */
