/*
 * Copyright (C) 2016 Fundacion Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32l1
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Francisco Molina <francisco.molina@inria.cl>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Nick v. IJzendoorn <nijzendoorn@engineering-spirit.nl>
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
 */
#define ADC_CLOCK_HIGH      (0)
#define ADC_CLOCK_MEDIUM    (ADC_CCR_ADCPRE_0)
#define ADC_CLOCK_LOW       (ADC_CCR_ADCPRE_1)

/**
 * @brief   ADC sample time, cycles
 */
#define ADC_SAMPLE_TIME_4C    (0)
#define ADC_SAMPLE_TIME_9C    (1)
#define ADC_SAMPLE_TIME_16C   (2)
#define ADC_SAMPLE_TIME_24C   (3)
#define ADC_SAMPLE_TIME_48C   (4)
#define ADC_SAMPLE_TIME_96C   (5)
#define ADC_SAMPLE_TIME_192C  (6)
#define ADC_SAMPLE_TIME_384C  (7)

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
 * All STM32l1 CPU's have single ADC device
 */
static mutex_t lock = MUTEX_INIT;

static inline void prep(void)
{
    mutex_lock(&lock);
    /* ADC clock is always HSI clock */
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

static void adc_set_sample_time(uint8_t time)
{
    uint8_t i;
    uint32_t reg32 = 0;

    for (i = 0; i <= 9; i++) {
        reg32 |= (time << (i * 3));
    }
#if !defined STM32L1XX_MD
    ADC1->SMPR0 = reg32;
#endif
    ADC1->SMPR1 = reg32;
    ADC1->SMPR2 = reg32;
    ADC1->SMPR3 = reg32;
}

int adc_init(adc_t line)
{
    /* check if the line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power-on the device */
    prep();

    /* configure the pin */
    if ((adc_config[line].pin != GPIO_UNDEF))
        gpio_init_analog(adc_config[line].pin);

    /* set ADC clock prescaler */
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CLOCK_MEDIUM;

    /* Set sample time */
    /* Min 4us needed for temperature sensor measurements */
    switch (ADC->CCR & ADC_CCR_ADCPRE) {
        case ADC_CLOCK_LOW:
            /* 4 MHz ADC clock -> 16 cycles */
            adc_set_sample_time(ADC_SAMPLE_TIME_16C);
            break;
        case ADC_CLOCK_MEDIUM:
            /* 8 MHz ADC clock -> 48 cycles */
            adc_set_sample_time(ADC_SAMPLE_TIME_48C);
            break;
        default:
            /* 16 MHz ADC clock -> 96 cycles */
            adc_set_sample_time(ADC_SAMPLE_TIME_96C);
    }

    /* check if this channel is an internal ADC channel, if so
     * enable the internal temperature and Vref */
    if (adc_config[line].chan == 16 || adc_config[line].chan == 17) {
        ADC->CCR |= ADC_CCR_TSVREFE;
    }

    /* enable the ADC module */
    ADC1->CR2 = ADC_CR2_ADON;
    /* turn off during idle phase*/
    ADC1->CR1 = ADC_CR1_PDI;

    /* free the device again */
    done();

    return 0;
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

    tmpreg1 |= align | edge | etrig | (continuous_conv_mode << 1);
    ADC1->CR2 = tmpreg1;

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
        uint16_t cal;
        if (get_cpu_category() < 3) {
            /* low-end devices doesn't provide calibration values, see errata */
            cal = 1672;
            
        } else {
            cal = *(uint16_t *)ADC_VREFINT_CAL;
        }
        sample = 3000 * (cal) / sample;
	}

	/* Chip temperature calculation */
	if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {

        uint16_t cal1, cal2;
        if (get_cpu_category() < 3) {
        /* low-end devices doesn't provide calibration values, see errata */
                cal1 = 670;
                cal2 = 848;
        } else {
                cal1 = *(uint16_t *)ADC_TS_CAL1;
                cal2 = *(uint16_t *)ADC_TS_CAL2;
        }
        /* Correct temperature sensor data for actual Vdd */
        sample_ts = (sample_ts * sample)/3000;
        
        /* Calculate chip temperature */
        /* sample = Vdd, sample_ts = temperature sensor data */
        /* 0.1 C resolution */
        sample_ts = 300 - (((int)cal1 - sample_ts)*80) / (int)(cal2 - cal1);
        
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