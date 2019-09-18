/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32f0
 * @ingroup     drivers_periph_adc
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

/* Factory calibration data */
#define ADC_VREFINT_CAL     (0x1FFFF7BAUL)
#define ADC_TSENSE_CAL1     (0x1FFFF7B8UL)

/**
 * @brief   Load the ADC configuration
 */
static const adc_conf_t adc_config[] = ADC_CONFIG;

/**
 * @brief   Allocate locks for all three available ADC device
 *
 * All STM32F0 CPUs we support so far only come with a single ADC device.
 */
static mutex_t lock = MUTEX_INIT;

static inline void prep(void)
{
    mutex_lock(&lock);
    periph_clk_en(APB2, RCC_APB2ENR_ADCEN);
}

static inline void done(void)
{
    periph_clk_dis(APB2, RCC_APB2ENR_ADCEN);
    mutex_unlock(&lock);
}

int adc_init(adc_t line)
{
    /* make sure the given line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power on the device */
    prep();
    
    /* ADC auto enable */
    ADC1->CFGR1 |= ADC_CFGR1_AUTOFF;
    
    /* reset configuration */
    ADC1->CFGR2 = 0;
    
    /* configure sampling time */
    ADC1->SMPR = 0x6;   /* 71.5 ADC clock cycles = 5 us */
    
    if (adc_config[line].pin != GPIO_UNDEF) {
        /*configure the pin */
        gpio_init_analog(adc_config[line].pin);
    }
    
    /* calibrate ADC */
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL) {}
    
    /* power off and release device for now */
    done();
    
    return 0;
}

int adc_sample(adc_t line,  adc_res_t res)
{
    int sample;
    int cal_vref, cal_ts1;

    /* check if resolution is applicable */
    if (res > 0xf0) {
        return -1;
    }
    
    /* lock and power on the ADC device  */
    prep();
    
    /* Reactivate VREFINT and temperature sensor if necessary */
    if ((adc_config[line].chan == ADC_VREF_CHANNEL) || (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL)) {
        ADC->CCR |= (ADC_CCR_VREFEN | ADC_CCR_TSEN);
        
        /* there's no PWR_CSR_VREFINTRDYF on STM32F030 and STM32F070 */
        #if defined(PWR_CSR_VREFINTRDYF)
            while ((PWR->CSR & PWR_CSR_VREFINTRDYF) == 0);
        #endif
    }
    
    /* set resolution and channel */
    ADC1->CFGR1 &= ~ADC_RES_6BIT;
    ADC1->CFGR1 |= res;
    ADC1->CHSELR = (1 << adc_config[line].chan);

    /* start conversion and wait for results */
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC)) {}
    /* read result */
    sample = (int)ADC1->DR;

    /* in case of temperature channel sample VDD too */
    int sample_vref = 0;
    if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {
        /* sample VREF */
        ADC1->CHSELR = (1 << ADC_VREF_CHANNEL);
        ADC1->CR |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC)) {}
        
        sample_vref = (int)ADC1->DR;
        
        /* calibrate temperature data */
        cal_ts1   = *(uint16_t *)ADC_TSENSE_CAL1;
        cal_vref  = *(uint16_t *)ADC_VREFINT_CAL;

        /* calibration values are for ADC_RES_12BIT, adjust for it if needed */
        switch (res) {
            case ADC_RES_6BIT:
                sample = sample << 6;
                sample_vref = sample_vref << 6;
                break;
            case ADC_RES_8BIT:
                sample = sample << 4;
                sample_vref = sample_vref << 4;
                break;
            case ADC_RES_10BIT:
                sample = sample << 2;
                sample_vref = sample_vref << 2;
                break;
            default:
                break;
        }
        
        /* Adjust temperature sensor data for actual VDD */
        sample = (cal_vref * sample)/sample_vref;

        /* return chip temperature, 1 C resolution */
        /* 4.3 mV/C, datasheet 6.3.17 -> 4.3 * 3300/4096 = 3.464 ADC counts per C */
        sample = 30 + (100*(sample - cal_ts1))/346;
    }
    
    /* Deactivate VREFINT and temperature sensor to save power */
    ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_TSEN);
    
    /* VDD calculation based on VREF */
	if (adc_config[line].chan == ADC_VREF_CHANNEL) {
        cal_vref = *(uint16_t *)ADC_VREFINT_CAL;
        
        /* calibration value is for ADC_RES_12BIT, adjust for it if needed */
        switch (res) {
            case ADC_RES_6BIT:
                sample = sample << 6;
                break;
            case ADC_RES_8BIT:
                sample = sample << 4;
                break;
            case ADC_RES_10BIT:
                sample = sample << 2;
                break;
            default:
                break;
        }
        
        /* return Vdd in mV instead of Vref in ADC counts*/
        sample = (3300 * cal_vref) / sample;
	}
    
    /* unlock and power off device again */
    done();
    
    return sample;
}
