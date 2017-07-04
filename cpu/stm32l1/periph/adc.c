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
 * @brief   ADC clock settings
 *
 * NB: with ADC_CLOCK_HIGH, Vdda should be 2.4V min
 *
 * @{
 */
#define ADC_CLOCK_HIGH      0x0
#define ADC_CLOCK_MEDIUM    ADC_CCR_ADCPRE_0
#define ADC_CLOCK_LOW       ADC_CCR_ADCPRE_1

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
    
    /* "The ADC clock which is always the HSI clock" */
    if (!(RCC->CR & RCC_CR_HSION)) {
        RCC->CR |= RCC_CR_HSION;
        /* Wait for HSI to become ready */
        while (!(RCC->CR & RCC_CR_HSION)) {}
    }
    
    periph_clk_en(APB2, RCC_APB2ENR_ADC1EN);
}

static inline void done(void)
{
    periph_clk_dis(APB2, RCC_APB2ENR_ADC1EN);

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
    /* configure the pin */
	/* no need to configure GPIO for ADC channels not connected to any GPIO */
	if ((adc_config[line].pin != GPIO_UNDEF)) {
		gpio_init_analog(adc_config[line].pin);
	}
    
    /* set ADC clock */

    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CLOCK_MEDIUM;

    /* Set sample time */
    /* Min 4us needed for temperature sensor measurements */
    switch (ADC->CCR & ADC_CCR_ADCPRE) {
        case ADC_CLOCK_LOW:
            /* 4 MHz ADC clock -> 16 cycles */
            adc_set_sample_time_on_all_channels(0b010);
            break;
        case ADC_CLOCK_MEDIUM:
            /* 8 MHz ADC clock -> 48 cycles */
            adc_set_sample_time_on_all_channels(0b100);
            break;
        default:
            /* 16 MHz ADC clock -> 92 cycles */
            adc_set_sample_time_on_all_channels(0b101);
    }
    
    /* power off and release device for now */
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

    if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {
        uint8_t channels[2] = { ADC_TEMPERATURE_CHANNEL, ADC_VREF_CHANNEL };
        adc_set_regular_sequence(2, channels);
        ADC1->CR1 |= ADC_CR1_SCAN;
        ADC1->CR2 &= ~ADC_CR2_DELS;
        ADC1->CR2 |= ADC_CR2_DELS_0;
    } else {
        uint8_t channels[1] = { (uint8_t) adc_config[line].chan };
        adc_set_regular_sequence(1, channels);
    }

    /* Enable ADC */
    ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;
    
	/* Enable temperature and Vref conversion */
	if (adc_config[line].pin == GPIO_UNDEF) {
		ADC->CCR |= ADC_CCR_TSVREFE;
        while ((PWR->CSR & PWR_CSR_VREFINTRDYF) == 0);
	}
	
	/* Wait for ADC to become ready */
	while ((ADC1->SR & ADC_SR_ADONS) == 0);
	
	ADC1->CR2 |= ADC_CR2_EOCS;

    /* Start conversion on regular channels. */
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;

    /* Wait until the end of ADC conversion */
    while ((ADC1->SR & ADC_SR_EOC) == 0);

    /* read result */
    sample = (int)ADC1->DR;
    
    int sample_ts = 0;
    if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {
        sample_ts = sample;
        while ((ADC1->SR & ADC_SR_EOC) == 0);
        sample = (int)ADC1->DR;
    }
    
	/* VDD calculation based on VREFINT */
	if ((adc_config[line].chan == ADC_VREF_CHANNEL) || (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL)) {
#if !defined (STM32L1XX_MDP) && !defined (STM32L1XX_HD) && !defined (STM32L1XX_XL)
/* low-end devices doesn't provide calibration values, see errata */
        uint16_t *cal;
        *cal = 1672;
#else
		uint16_t *cal = ADC_VREFINT_CAL;
#endif
		sample = 3000 * (*cal) / sample;
	}

	/* Chip temperature calculation */
	if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {

#if !defined (STM32L1XX_MDP) && !defined (STM32L1XX_HD) && !defined (STM32L1XX_XL)
/* low-end devices doesn't provide calibration values, see errata */
        uint16_t *cal1, *cal2;
        *cal1 = 670;
        *cal2 = 848;
#else
        uint16_t *cal1 = ADC_TS_CAL1;
		uint16_t *cal2 = ADC_TS_CAL2;
#endif
        /* Correct temperature sensor data for actual Vdd */
        sample_ts = (sample_ts * sample)/3000;
        
        /* Calculate chip temperature */
        /* sample = Vdd, sample_ts = temperature sensor data */
        /* 0.1 C resolution */
        sample_ts = 300 - (((int)*cal1 - sample_ts)*80) / (int)(*cal2 - *cal1);
        
        ADC1->CR1 &= ~ADC_CR1_SCAN;
        ADC1->CR2 &= ~ADC_CR2_DELS;
	}

	/* Disable temperature and Vref conversion */
	ADC->CCR &= ~ADC_CCR_TSVREFE;
	
    /* unlock and power off device */
	ADC1->CR2 &= ~(ADC_CR2_ADON);
    done();

    return sample;
}