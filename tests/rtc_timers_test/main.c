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
 * @brief       minimal RIOT application to test RTC timers
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
#include "mutex.h"

#include "rtc-timers.h"

#include "sx1276.h"

/**
 * @brief Time delay before RTC alarm
 */

#define SLEEP_TIME_SEC 5


/**
 * @name SX1276 configuration
 * @{
 */
#define RF_FREQUENCY                                868900000   // Hz, 868.9MHz
#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER                             10          // dBm

#define LORA_PREAMBLE_LENGTH                        8           // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         10          // Symbols

#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION                           false

#define SX1276_DIO0 GPIO_PIN(PORT_A, 8)
#define SX1276_DIO1 GPIO_PIN(PORT_A, 9)
#define SX1276_DIO2 GPIO_PIN(PORT_A, 10)
#define SX1276_DIO3 GPIO_PIN(PORT_A, 11)

#define SX1276_RESET GPIO_PIN(PORT_C, 6)

/** SX1276 SPI */

#define USE_SPI_0

#ifdef USE_SPI_1
#define SX1276_SPI SPI_1
#define SX1276_SPI_NSS GPIO_PIN(PORT_B, 12)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

#ifdef USE_SPI_0
#define SX1276_SPI SPI_0
#define SX1276_SPI_NSS GPIO_PIN(PORT_A, 4)
#define SX1276_SPI_MODE SPI_CONF_FIRST_RISING
#define SX1276_SPI_SPEED SPI_SPEED_1MHZ
#endif

#endif

static sx1276_t sx1276;

static void init_sx1276(void) {
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
    settings.modem = SX1276_MODEM_LORA;
    settings.state = SX1276_RF_IDLE;

    sx1276.settings = settings;

    sx1276.sx1276_event_cb = NULL;

    /* Launch initialization of driver and device */
    puts("init_radio: initializing driver...");
    sx1276_init(&sx1276);

    /* Put chip into sleep */
    sx1276_set_sleep(&sx1276);
}

int main(void)
{
    lpm_prevent_sleep = 1;

	rtc_init();
	rtctimers_init();
	xtimer_init();

	init_sx1276();

	lpm_prevent_sleep = 0;

	while (1) {
		LED0_OFF;

		printf("Sleeping %u seconds\n", (unsigned int) SLEEP_TIME_SEC);

		rtctimers_sleep(SLEEP_TIME_SEC);

		puts("Woke up");
		LED0_ON;

		lpm_prevent_sleep = 1;
		xtimer_sleep(1);
		lpm_prevent_sleep = 0;
	}

    return 0;
}
