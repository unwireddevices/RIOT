/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"

/**
 * @brief   Maximum allowed ADC clock speed
 */
#define MAX_ADC_SPEED           (12000000U)

/**
 * @brief   Load the ADC configuration
 * @{
 */
#ifdef ADC_CONFIG
static const adc_conf_t adc_config[] = ADC_CONFIG;
#else
static const adc_conf_t adc_config[] = {};
#endif

/**
 * @brief   Allocate locks for all three available ADC device
 *
 * All STM32L1 CPUs we support so far only come with a single ADC device.
 */
static mutex_t lock = MUTEX_INIT;

static inline void prep(void)
{
    mutex_lock(&lock);
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
   // ADC1->CR2 |= ADC_CR2_ADON;
}

static inline void done(void)
{
    RCC->APB2ENR &= ~(RCC_APB2ENR_ADC1EN);
    //ADC1->CR2 &= ~(ADC_CR2_ADON);

    mutex_unlock(&lock);
}

void adc_set_sample_time_on_all_channels(uint8_t time)
{
	uint8_t i;
	uint32_t reg32 = 0;

	for (i = 0; i <= 9; i++) {
		reg32 |= (time << (i * 3));
	}

	ADC1->SMPR0 = reg32;
	ADC1->SMPR1 = reg32;
	ADC1->SMPR2 = reg32;
	ADC1->SMPR3 = reg32;
}

int adc_init(adc_t line)
{
    /* make sure the given line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power on the device */
    prep();
    /*configure the pin */
    gpio_init_analog(adc_config[line].pin);

    /* Set sample time */
    adc_set_sample_time_on_all_channels(0x03 /* 25 cycles */);

    /* power off an release device for now */
    done();

    return 0;
}

void adc_set_regular_sequence(uint8_t length, uint8_t channel[])
{
	uint32_t fifth6 = 0;
	uint32_t fourth6 = 0;
	uint32_t third6 = 0;
	uint32_t second6 = 0;
	uint32_t first6 = 0;
	uint8_t i = 0;

	if (length > 20) {
		return;
	}

	for (i = 1; i <= length; i++) {
		if (i <= 6) {
			first6 |= (channel[i - 1] << ((i - 1) * 5));
		}
		if ((i > 6) & (i <= 12)) {
			second6 |= (channel[i - 1] << ((i - 6 - 1) * 5));
		}
		if ((i > 12) & (i <= 18)) {
			third6 |= (channel[i - 1] << ((i - 12 - 1) * 5));
		}
		if ((i > 18) & (i <= 24)) {
			fourth6 |= (channel[i - 1] << ((i - 18 - 1) * 5));
		}
		if ((i > 24) & (i <= 28)) {
			fifth6 |= (channel[i - 1] << ((i - 24 - 1) * 5));
		}
	}

	ADC1->SQR1 = fifth6 | ((length - 1) << 20);
	ADC1->SQR2 = fourth6;
	ADC1->SQR3 = third6;
	ADC1->SQR4 = second6;
	ADC1->SQR5 = first6;

}

#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)

int adc_sample(adc_t line,  adc_res_t res)
{
    int sample;

    /* lock and power on the ADC device  */
    prep();

    /* ADC1 CR1 Configuration */
    uint32_t tmpreg1 = ADC1->CR1;
    tmpreg1 &= CR1_CLEAR_MASK;
    tmpreg1 |= res;
    ADC1->CR1 = tmpreg1;

    /* CR2 config */
    tmpreg1 = ADC1->CR2;
    tmpreg1 &= CR2_CLEAR_MASK;

    uint32_t align = 0x00000000; /* Left alignment of data */
    uint32_t edge = 0x00000000;
    uint32_t etrig = 0x03000000;
    uint32_t continuous_conv_mode = 0;

    tmpreg1 |= (uint32_t)(align | edge | etrig | ((uint32_t)continuous_conv_mode << 1));
    ADC1->CR2 = tmpreg1;

    uint8_t channels[1] = { (uint8_t) adc_config[line].chan };
    adc_set_regular_sequence(1, channels);

    ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;

	/* Start conversion on regular channels. */
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

	/* Wait until the end of ADC conversion */
	while ((ADC1->SR & ADC_SR_EOC) == 0);

    /* read result */
    sample = (int)ADC1->DR;

    /* unlock and power off device */
    done();

    return sample;
}
