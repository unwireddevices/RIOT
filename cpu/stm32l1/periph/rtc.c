/*
 * Copyright (C) 2015 Lari Lehtomäki
 * Copyright (C) 2016 Laksh Bhatia
 * Copyright (C) 2017 Unwired Devices LLC [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @{
 * @file
 * @brief       Low-level RTC driver implementation
 * @author      Lari Lehtomäki <lari@lehtomaki.fi>
 * @author      Laksh Bhatia <bhatialaksh3@gmail.com>
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @}
 */


#include <time.h>
#include <string.h>
#include "cpu.h"
#include "periph/rtc.h"
#include "periph_conf.h"

/* guard file in case no RTC device was specified */
#if RTC_NUMOF

#define RTC_WRITE_PROTECTION_KEY1   (0xCA)
#define RTC_WRITE_PROTECTION_KEY2   (0x53)
#define RTC_SYNC_PRESCALER          (0xff)  /**< prescaler for 32.768 kHz oscillator */
#define RTC_ASYNC_PRESCALER         (0x7f)  /**< prescaler for 32.768 kHz oscillator */

#define MCU_YEAR_OFFSET              (100)  /**< struct tm counts years since 1900
                                                but RTC has only two-digit year
                                                hence the offset of 100 years. */

typedef struct {
    rtc_alarm_cb_t cb;          /**< callback called from RTC interrupt */
    rtc_wkup_cb_t wkup_cb;      /**< Wake up timer callback */
    rtc_alarm_cb_t millis_cb;       /**< Subseconds alarm callback */

    void *arg;                  /**< argument passed to the callback */
    void *wkup_arg;             /**< argument passed to wakeup callback */
    void *millis_arg;               /**< argument passed to subseconds alarm callback */
} rtc_state_t;

static rtc_state_t rtc_callback;

static uint8_t byte2bcd(uint8_t value);

/**
 * @brief Initializes the RTC to use LSE (external 32.768 kHz oscillator) as a
 * clock source. Verify that your board has this oscillator. If other clock
 * source is used, then also the prescaler constants should be changed.
 */
void rtc_init(void)
{
    rtc_poweron();

    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;

    /* Enter RTC Init mode */
    RTC->ISR = 0;
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0) ;

    /* Set 24-h clock */
    RTC->CR &= ~RTC_CR_FMT;
    /* Timestamps enabled */
    RTC->CR |= RTC_CR_TSE;

    /* Configure the RTC PRER */
    RTC->PRER = RTC_SYNC_PRESCALER;
    RTC->PRER |= (RTC_ASYNC_PRESCALER << 16);

    /* Exit RTC init mode */
    RTC->ISR &= (uint32_t) ~RTC_ISR_INIT;

    /* Enable RTC write protection */
    RTC->WPR = 0xff;
}

int rtc_set_time(struct tm *time)
{
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;

    /* Enter RTC Init mode */
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0) ;


    RTC->DR = ((((uint32_t)byte2bcd(time->tm_year - MCU_YEAR_OFFSET) << 16) & (RTC_DR_YT | RTC_DR_YU)) |
               (((uint32_t)byte2bcd(time->tm_mon + 1) <<  8) & (RTC_DR_MT | RTC_DR_MU)) |
               (((uint32_t)byte2bcd(time->tm_mday) <<  0) & (RTC_DR_DT | RTC_DR_DU)));

    RTC->TR = ((((uint32_t)byte2bcd(time->tm_hour) << 16) & (RTC_TR_HT | RTC_TR_HU)) |
               (((uint32_t)byte2bcd(time->tm_min) <<  8) & (RTC_TR_MNT | RTC_TR_MNU)) |
               (((uint32_t)byte2bcd(time->tm_sec) <<  0) & (RTC_TR_ST | RTC_TR_SU)));

    /* Exit RTC init mode */
    RTC->ISR &= (uint32_t) ~RTC_ISR_INIT;
    /* Enable RTC write protection */
    RTC->WPR = RTC_WPR_KEY;
    return 0;
}

int rtc_get_time(struct tm *time)
{
    time->tm_year = MCU_YEAR_OFFSET;
    
    /* clear RSF bit */
    RTC->ISR &= ~RTC_ISR_RSF;
    
    /* wait for RSF to be set by hardware */
    while (!(RTC->ISR & RTC_ISR_RSF)) {}
    
    /* RTC registers need to be read at least twice when running at f < 32768*7 = 229376 Hz APB1 clock */
    /* reading TR locks registers so it must be read first, DR must be read last */
    uint32_t rtc_time_reg = RTC->TR;

    /* second read */
    if (RTC->TR != rtc_time_reg) {
        /* 3rd read if 1st and 2nd don't match */
        rtc_time_reg = RTC->TR;
    }
    
    uint32_t rtc_date_reg = RTC->DR;
    
    time->tm_year += (((rtc_date_reg & RTC_DR_YT)  >> 20) * 10) + ((rtc_date_reg & RTC_DR_YU)  >> 16);
    time->tm_mon  = (((rtc_date_reg & RTC_DR_MT)  >> 12) * 10) + ((rtc_date_reg & RTC_DR_MU)  >>  8) - 1;
    time->tm_mday = (((rtc_date_reg & RTC_DR_DT)  >>  4) * 10) + ((rtc_date_reg & RTC_DR_DU)  >>  0);
    time->tm_wday = ((rtc_date_reg & RTC_DR_WDU)  >>  13);
    /* tm_wday should be days since Sunday, so it's 0 if today is Sunday */
    /* STM32 returns day of week instead, so Monday is 1 and Sunday is 7 */
    if (time->tm_wday == 7) {
        time->tm_wday = 0;
    }
    time->tm_hour = (((rtc_time_reg & RTC_TR_HT)  >> 20) * 10) + ((rtc_time_reg & RTC_TR_HU)  >> 16);
    if ((rtc_time_reg & RTC_TR_PM) && (RTC->CR & RTC_CR_FMT)) {
        time->tm_hour += 12;
    }
    time->tm_min  = (((rtc_time_reg & RTC_TR_MNT) >> 12) * 10) + ((rtc_time_reg & RTC_TR_MNU) >>  8);
    time->tm_sec  = (((rtc_time_reg & RTC_TR_ST)  >>  4) * 10) + ((rtc_time_reg & RTC_TR_SU)  >>  0);
    return 0;
}

int rtc_set_alarm(struct tm *time, rtc_alarm_cb_t cb, void *arg)
{
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;

    RTC->CR &= ~(RTC_CR_ALRAE);
    while ((RTC->ISR & RTC_ISR_ALRAWF) == 0) ;
    
    /* seconds, minutes, hours and day must match */
    RTC->ALRMAR &= ~(RTC_ALRMAR_MSK1 | RTC_ALRMAR_MSK2 | RTC_ALRMAR_MSK3 | RTC_ALRMAR_MSK4);
    /* 24 hrs format and week day instead of day number */
    RTC->ALRMAR |= RTC_ALRMAR_PM | RTC_ALRMAR_WDSEL;
    
    RTC->ALRMAR = ((((uint32_t)byte2bcd(time->tm_wday) << 24) & (RTC_ALRMAR_DT | RTC_ALRMAR_DU)) |
                   (((uint32_t)byte2bcd(time->tm_hour) << 16) & (RTC_ALRMAR_HT | RTC_ALRMAR_HU)) |
                   (((uint32_t)byte2bcd(time->tm_min) <<  8) & (RTC_ALRMAR_MNT | RTC_ALRMAR_MNU)) |
                   (((uint32_t)byte2bcd(time->tm_sec) <<  0) & (RTC_ALRMAR_ST | RTC_ALRMAR_SU)));
    /* Enable Alarm A */
    RTC->CR |= RTC_CR_ALRAE;
    RTC->CR |= RTC_CR_ALRAIE;
    RTC->ISR &= ~(RTC_ISR_ALRAF);

    /* Enable RTC write protection */
    RTC->WPR = 0xFF;

    EXTI->IMR  |= EXTI_IMR_MR17;
    EXTI->RTSR |= EXTI_RTSR_TR17;
    NVIC_SetPriority(RTC_Alarm_IRQn, RTC_IRQ_PRIO);
    NVIC_EnableIRQ(RTC_Alarm_IRQn);

    rtc_callback.cb = cb;
    rtc_callback.arg = arg;

    return 0;
}

int rtc_get_alarm(struct tm *time)
{
    time->tm_year = MCU_YEAR_OFFSET;
    time->tm_year += (((RTC->DR     & RTC_DR_YT)      >> 20) * 10) + ((RTC->DR & RTC_DR_YU)          >> 16);
    time->tm_mon  = (((RTC->DR     & RTC_DR_MT)      >> 12) * 10) + ((RTC->DR & RTC_DR_MU)          >>  8) - 1;
    time->tm_wday = (((RTC->ALRMAR & RTC_ALRMAR_DT)  >> 28) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_DU)  >> 24);
    time->tm_hour = (((RTC->ALRMAR & RTC_ALRMAR_HT)  >> 20) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_HU)  >> 16);
    if ((RTC->ALRMAR & RTC_ALRMAR_PM) && (RTC->CR & RTC_CR_FMT)) {
        time->tm_hour += 12;
    }
    time->tm_min  = (((RTC->ALRMAR & RTC_ALRMAR_MNT) >> 12) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_MNU) >>  8);
    time->tm_sec  = (((RTC->ALRMAR & RTC_ALRMAR_ST)  >>  4) * 10) + ((RTC->ALRMAR & RTC_ALRMAR_SU)  >>  0);
    return 0;
}

void rtc_clear_alarm(void)
{
    /* Disable Alarm A */
    
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
    
    RTC->CR &= ~RTC_CR_ALRAE;
    RTC->CR &= ~RTC_CR_ALRAIE;
    
    RTC->WPR = 0xFF;

    rtc_callback.cb = NULL;
    rtc_callback.arg = NULL;
}

int rtc_millis_set_alarm(int milliseconds, rtc_alarm_cb_t cb, void *arg)
{   
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;

    RTC->CR &= ~(RTC_CR_ALRBE);
    while ((RTC->ISR & RTC_ISR_ALRBWF) == 0) ;
    
    /* setting seconds */
    uint8_t seconds = milliseconds/1000;
    RTC->ALRMBR = (((uint32_t)byte2bcd(seconds) & (RTC_ALRMAR_ST | RTC_ALRMAR_SU)));
    
    /* minutes, hours and date doesn't matter */
    RTC->ALRMBR |= (RTC_ALRMBR_MSK2 | RTC_ALRMBR_MSK3 | RTC_ALRMBR_MSK4);

    uint32_t msec = milliseconds % 1000;
    uint32_t alarm_millis_time = 255 - (msec*1000)/3922;
    
    /* set up subseconds alarm */
    uint32_t regalarm = RTC->ALRMBSSR;
    regalarm |= (0x8 << 24); // compare 8 bits only
    regalarm &= ~(RTC_ALRMBSSR_SS);
    regalarm |= (alarm_millis_time & 0xFF);
    RTC->ALRMBSSR = regalarm;
    
    /* Enable Alarm B */
    RTC->CR |= RTC_CR_ALRBE;
    RTC->CR |= RTC_CR_ALRBIE;
    RTC->ISR &= ~(RTC_ISR_ALRBF);
    
    /* Enable RTC write protection */
    RTC->WPR = 0xFF;

    EXTI->IMR  |= EXTI_IMR_MR17;
    EXTI->RTSR |= EXTI_RTSR_TR17;
    NVIC_SetPriority(RTC_Alarm_IRQn, RTC_IRQ_PRIO);
    NVIC_EnableIRQ(RTC_Alarm_IRQn);

    rtc_callback.millis_cb = cb;
    rtc_callback.millis_arg = arg;

    return 0;
}

void rtc_millis_clear_alarm(void)
{
    /* Disable Alarm B */
    
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
    
    RTC->CR &= ~RTC_CR_ALRBE;
    RTC->CR &= ~RTC_CR_ALRBIE;
    
    RTC->WPR = 0xFF;

    rtc_callback.millis_cb = NULL;
    rtc_callback.millis_arg = NULL;
}

int rtc_millis_get_time(uint32_t *millis)
{
    /* clear RSF bit */
    RTC->ISR &= ~RTC_ISR_RSF;
    
    /* wait for RSF to be set by hardware */
    while (!(RTC->ISR & RTC_ISR_RSF)) {}
    
    /* RTC registers need to be read at least twice when running at f < 32768*7 = 229376 Hz APB1 clock */
    uint32_t rtc_ssr_counter = RTC->SSR;

    /* second read */
    if (RTC->SSR != rtc_ssr_counter) {
        /* 3rd read if 1st and 2nd don't match */
        rtc_ssr_counter = RTC->SSR;
    }
    
    uint32_t milliseconds = ((255 - (rtc_ssr_counter & 0xFF))*3922)/1000;
    
    /* clear RSF bit */
    RTC->ISR &= ~RTC_ISR_RSF;
    
    /* wait for RSF to be set by hardware */
    while (!(RTC->ISR & RTC_ISR_RSF)) {}
    
    /* RTC registers need to be read at least twice when running at f < 32768*7 = 229376 Hz APB1 clock */
    /* reading TR locks registers so it must be read first, DR must be read last */
    uint32_t rtc_time_reg = RTC->TR;

    /* second read */
    if (RTC->TR != rtc_time_reg) {
        /* 3rd read if 1st and 2nd don't match */
        rtc_time_reg = RTC->TR;
    }
    
    RTC->DR;
    
    uint32_t seconds  = (((rtc_time_reg & RTC_TR_ST)  >>  4) * 10) + ((rtc_time_reg & RTC_TR_SU)  >>  0);
    
    *millis = milliseconds + 1000*seconds;

    /* unlock RTC registers by reading DR */
    rtc_ssr_counter = RTC->DR;
    
    return 0;
}

int rtc_set_wakeup(uint32_t period_us, rtc_wkup_cb_t cb, void *arg)
{
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Unlock RTC write protection */
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
    
    /* Disable periodic wakeup */
    RTC->CR &= ~(RTC_CR_WUTE);
    while ((RTC->ISR & RTC_ISR_WUTWF) == 0) ;
    
    /* Set wakeup timer value */
    period_us = ((period_us * 100)/12207) - 1;   
    RTC->WUTR = (period_us & 0xFFFF);
    
    /* Set wakeup timer clock source to RTCCLK/4 */
    /* Min period 244 us, maximum 8 s */
    RTC->CR &= ~(RTC_CR_WUCKSEL);
    RTC->CR |= (RTC_CR_WUCKSEL_1);
    
    /* Enable periodic wakeup */
    RTC->CR |= RTC_CR_WUTE;
    RTC->CR |= RTC_CR_WUTIE;
    RTC->ISR &= ~(RTC_ISR_WUTF);

    /* Enable RTC write protection */
    RTC->WPR = 0xFF;

    EXTI->IMR  |= EXTI_IMR_MR20;
    EXTI->RTSR |= EXTI_RTSR_TR20;
    NVIC_SetPriority(RTC_WKUP_IRQn, RTC_IRQ_PRIO);
    NVIC_EnableIRQ(RTC_WKUP_IRQn);

    rtc_callback.wkup_cb = cb;
    rtc_callback.wkup_arg = arg;

    return 0;
}

void rtc_enable_wakeup(void) {
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;
    /* Enable wakeup */
    RTC->CR |= RTC_CR_WUTE;
    RTC->WPR = 0xFF;
}

void rtc_disable_wakeup(void)
{
    RTC->WPR = RTC_WRITE_PROTECTION_KEY1;
    RTC->WPR = RTC_WRITE_PROTECTION_KEY2;   
    /* Disable wakeup */
    RTC->CR &= ~(RTC_CR_WUTE);
    RTC->WPR = 0xFF;
}

int rtc_save_backup(uint32_t data, uint8_t reg_num) {    
    __IO uint32_t tmp = 0;
    
    tmp = RTC_BASE + 0x50;
    tmp += (reg_num * 4);

    /* Write the specified register */
    *(__IO uint32_t *)tmp = (uint32_t)data;
    
    return 0;
}

uint32_t rtc_restore_backup(uint8_t reg_num) {
    __IO uint32_t tmp = 0;
    
    tmp = RTC_BASE + 0x50;
    tmp += (reg_num * 4);

    /* Read the specified register */
    return (*(__IO uint32_t *)tmp);
}

void rtc_poweron(void)
{
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Reset RTC domain - disabled as it clears backup registers */
    /*
    RCC->CSR |= RCC_CSR_RTCRST;
    RCC->CSR &= ~(RCC_CSR_RTCRST);
    */

    /* Enable the LSE clock (external 32.768 kHz oscillator) */
    RCC->CSR &= ~(RCC_CSR_LSEON);
    RCC->CSR &= ~(RCC_CSR_LSEBYP);
    RCC->CSR |= RCC_CSR_LSEON;
    while ((RCC->CSR & RCC_CSR_LSERDY) == 0) ;

    /* Switch RTC to LSE clock source */
    RCC->CSR &= ~(RCC_CSR_RTCSEL);
    RCC->CSR |= RCC_CSR_RTCSEL_LSE;

    /* Enable the RTC */
    RCC->CSR |= RCC_CSR_RTCEN;
}

void rtc_poweroff(void)
{
    /* Enable write access to RTC registers */
    periph_clk_en(APB1, RCC_APB1ENR_PWREN);
    PWR->CR |= PWR_CR_DBP;

    /* Reset RTC domain */
    RCC->CSR |= RCC_CSR_RTCRST;
    RCC->CSR &= ~(RCC_CSR_RTCRST);
    /* Disable the RTC */
    RCC->CSR &= ~RCC_CSR_RTCEN;
    /* Disable LSE clock */
    RCC->CSR &= ~(RCC_CSR_LSEON);
}

void isr_rtc_alarm(void)
{    
    if (RTC->ISR & RTC_ISR_ALRAF) {
        RTC->ISR &= ~RTC_ISR_ALRAF;
        if (rtc_callback.cb) {        
            rtc_callback.cb(rtc_callback.arg);
        }
    }
    
    if (RTC->ISR & RTC_ISR_ALRBF) {
        RTC->ISR &= ~RTC_ISR_ALRBF;
        if (rtc_callback.millis_cb) {
            rtc_callback.millis_cb(rtc_callback.millis_arg);
        }
    }
    
    EXTI->PR = EXTI_PR_PR17;
    
    cortexm_isr_end();
}


void isr_rtc_wkup(void)
{
    if (RTC->ISR & RTC_ISR_WUTF) {
        RTC->ISR &= ~RTC_ISR_WUTF;
        if (rtc_callback.wkup_cb != NULL) {
            rtc_callback.wkup_cb(rtc_callback.wkup_arg);
        }
    }
    
    EXTI->PR = EXTI_PR_PR20;
    
    cortexm_isr_end();
}

/**
 * Convert a number from unsigned to BCD
 *
 * @param[in] value to be converted
 * @return BCD representation of the value
 */
static uint8_t byte2bcd(uint8_t value)
{
    uint8_t bcdhigh = 0;

    while (value >= 10) {
        bcdhigh++;
        value -= 10;
    }

    return  ((uint8_t)(bcdhigh << 4) | value);
}

#endif /* RTC_NUMOF */
