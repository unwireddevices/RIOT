/*
 * Copyright (C) 2016 Cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     test
 * @{
 *
 * @file
 * @brief       minimal RIOT application to test entrance into LPM and waking-up by RTC alarm
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>

#include "lpm.h"
#include "periph/rtc.h"
#include "arch/lpm_arch.h"
#include "periph/gpio.h"
#include "thread.h"
#include "xtimer.h"

#include "board.h"
#include "sx1276.h"

#include "stm32l1xx.h"

static sx1276_t sx1276;

void radio_init(void)
{
    sx1276.nss_pin = SX1276_SPI_NSS;
    sx1276.spi = SX1276_SPI;

    sx1276.dio0_pin = SX1276_DIO0;
    sx1276.dio1_pin = SX1276_DIO1;
    sx1276.dio2_pin = SX1276_DIO2;
    sx1276.dio3_pin = SX1276_DIO3;

    sx1276.dio4_pin = (gpio_t) NULL;
    sx1276.dio5_pin = (gpio_t) NULL;
    sx1276.reset_pin = (gpio_t) SX1276_RESET;

    sx1276_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = MODEM_LORA;
    settings.state = RF_IDLE;

    sx1276.settings = settings;

    sx1276_init(&sx1276);

    sx1276_set_sleep(&sx1276);

    puts("init_radio: sx1276 initialization done");
}

/**
 * @brief Time delay before RTC alarm
 */

#define SLEEP_TIME_SEC 1
void gcb(void *arg)
{
}

/*
static void disable_gpios(void) {
    //RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |RCC_AHBENR_GPIOHEN;

    GPIOC->PUPDR=0;
    RCC->AHBENR &= ~RCC_AHBENR_GPIOCEN;
}*/

int main(void)
{
	lpm_prevent_sleep = 1;
    puts("= STM32 Low Power mode test =");

    xtimer_init();
    rtc_init();

    gpio_init_int(GPIO_PIN(PORT_C, 1), GPIO_IN, GPIO_RISING, gcb, NULL);

    volatile int i = 0;
    for(i = 0; i < 10000000; i++) ;

    //disable_gpios();

    puts("Disabling sx1276...");
    //radio_init();

    puts("Entering LPM...");

    /*
    struct tm t;
    rtc_get_time(&t);

    t.tm_sec += 10;
    rtc_set_alarm(&t, gcb, NULL);*/

    //lpm_prevent_sleep = 0;
	//lpm_set(LPM_POWERDOWN);

    lpm_prevent_sleep = 0;

    xtimer_sleep(10);
    printf("3. TIM5->PSC: %08x\n", TIM5->PSC);

	/* This code is supposed to execute after wake-up */
	LED0_TOGGLE;
	puts("Normally running");
	lpm_prevent_sleep = 1;

	xtimer_sleep(10);
	puts("After 10 seconds");

    while (1) ;

    return 0;
}
