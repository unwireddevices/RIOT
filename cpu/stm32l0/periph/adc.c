/*
 * Copyright (C) 2014-2017 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_stm32l0
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Aurélien Fillau <aurelien.fillau@we-sens.com>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph/pm.h"

/* Factory calibration data */
#define ADC_VREFINT_CAL     (0x1FF80078UL)
#define ADC_TSENSE_CAL1     (0x1FF8007AUL)
#define ADC_TSENSE_CAL2     (0x1FF8007EUL)

/**
 * @brief   Maximum allowed ADC clock speed
 */
#define MAX_ADC_SPEED           (12000000U)

/**
 * @brief   Allocate locks for all three available ADC device
 *
 * All STM32L0 CPUs we support so far only come with a single ADC device.
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

static void _enable_adc(void)
{
    /* Clear flag */
    ADC1->ISR |= ADC_ISR_ADRDY;

    /* enable device */
    ADC1->CR = ADC_CR_ADVREGEN | ADC_CR_ADEN;

    /* Wait for ADC to be ready */
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {}
}

static void _disable_adc(void)
{
    /* Stop any ongoing conversion */
    ADC1->CR |= ADC_CR_ADSTP;
    while (ADC1->CR & ADC_CR_ADSTP) {}
    
    /* Disable ADC */
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADEN) {}

    ADC1->ISR = 0;
}

int adc_init(adc_t line)
{
    /* make sure the given line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power on the device */
    prep();

    if (adc_config[line].pin != GPIO_UNDEF) {
        /* configure the pin */
        gpio_init_analog(adc_config[line].pin);
    }

    /* no watchdog, no discontinuous mode, no auto off, single conv, no trigger,
     * right align, 12bits, no dma, no wait */
    ADC1->CFGR1 = 0;
    /* no oversampling: Watch out, MSB (CKMODE) MUST not be changed while on
     * (it is zero by default) */
    ADC1->CFGR2 = 0;
    /* activate VREF, and set prescaler to 4 (4Mhz clock)
     * activate also temp sensor, so that it will be ready for temp measure */
    ADC->CCR = ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_PRESC_1;
    /* Sampling time selection: 7 => 160 clocks => 40µs @ 4MHz
     * (must be 10+10 for ref start and sampling time) */
    ADC1->SMPR |= ADC_SMPR_SMP;
    /* clear previous flag */
    ADC1->ISR |= ADC_ISR_EOC;
    
    if ((ADC1->CR & ADC_CR_ADEN) != 0) {
        ADC1->CR |= ADC_CR_ADDIS;
        while(ADC1->CR & ADC_CR_ADEN) {} /* Wait for ADC disabled */
    }
    
    if ((ADC1->CFGR1 & ADC_CFGR1_DMAEN) != 0) {
        while(ADC1->CFGR1 & ADC_CFGR1_DMAEN) {} /* Wait for DMA disabled */
    }

    /* Then, start a calibration */
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL) {} /* Wait for the end of calibration */

    /* power off and release device for now */
    done();

    return 0;
}

int adc_sample(adc_t line,  adc_res_t res)
{
    int sample;
    int cal_vref, cal_ts1, cal_ts2;

    /* check if resolution is applicable */
    if ( (res != ADC_RES_6BIT) &&
         (res != ADC_RES_8BIT) &&
         (res != ADC_RES_10BIT) &&
         (res != ADC_RES_12BIT)) {
        return -1;
    }

    /* lock and power on the ADC device  */
    prep();

    /* Enable ADC */
    _enable_adc();

    /* Reactivate VREFINT and temperature sensor if necessary */
    if ((adc_config[line].chan == ADC_VREF_CHANNEL) || (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL)) {
        ADC->CCR = (ADC_CCR_VREFEN | ADC_CCR_TSEN);
        while ((PWR->CSR & PWR_CSR_VREFINTRDYF) == 0);
    }
    
    /* set resolution and channel */
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;
    ADC1->CFGR1 |= res & ADC_CFGR1_RES;
    ADC1->CHSELR = (1 << adc_config[line].chan);

    /* clear flag */
    ADC1->ISR |= ADC_ISR_EOC;

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
        ADC1->ISR |= ADC_ISR_EOC;
        ADC1->CR |= ADC_CR_ADSTART;
        while (!(ADC1->ISR & ADC_ISR_EOC)) {}
        
        sample_vref = (int)ADC1->DR;
        
        /* calibrate temperature data */
        cal_ts1   = *(uint16_t *)ADC_TSENSE_CAL1;
        cal_ts2   = *(uint16_t *)ADC_TSENSE_CAL2;
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
        sample = 30 + (100*(sample - cal_ts1))/(cal_ts2 - cal_ts1);
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
        sample = (3000 * cal_vref) / sample;
	}
    
    /* Disable ADC */
    _disable_adc();

    /* unlock and power off device again */
    done();

    return sample;
}

static adc_cb_t adc_dma_callback;

int adc_sampling_start(adc_t line, adc_res_t res, uint16_t *buf, uint16_t wsize, adc_cb_t adc_cb, adc_conconv_mode_t mode)
{
    /* check if resolution is applicable */
    if ( (res != ADC_RES_6BIT) &&
         (res != ADC_RES_8BIT) &&
         (res != ADC_RES_10BIT) &&
         (res != ADC_RES_12BIT)) {
        return -1;
    }
    
    if ((!buf) || (!wsize)) {
        return -1;
    }
    
    /* block STOP mode */
    pm_block(PM_SLEEP);

    adc_dma_callback = adc_cb;

    /* lock and power on the ADC device  */
    prep();
    _enable_adc();

    /* disable DMA */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
    
    /* enable DMA clock */
    periph_clk_en(AHB, RCC_AHBENR_DMA1EN);
    /* disable DMA channel */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    
    /* set resolution and channel */
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;
    ADC1->CFGR1 |= res & ADC_CFGR1_RES;
    
    /* set trigger event: hardware trigger rising edge */
    ADC1->CFGR1 &= ~ADC_CFGR1_EXTSEL;
    ADC1->CFGR1 |= ((uint32_t)adc_config[line].trigger << 6);
    ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN;
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;
    
    /* enable DMA */
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
    
    /* setup DMA channel 1 */
    DMA1_Channel1->CCR = 0;
    /* high priority */
    DMA1_Channel1->CCR |= DMA_CCR_PL_1;
    /* 16-bit memory size */
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
    /* 16-bit peripheral size */
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
    /* memory increment mode */
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    /* transfer completed IRQ */
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    /* number of data */
    DMA1_Channel1->CNDTR = wsize;
    /* peripheral address */
    DMA1_Channel1->CPAR = (uint32_t)(&ADC1->DR);
    /* memory address */
    DMA1_Channel1->CMAR = (uint32_t)buf;
    
    /* disable interrupt */
    ADC1->IER &= ~ADC_IER_EOCIE;
    
    if (mode == ADC_CONTINUOUS_CIRCULAR) {
        ADC1->CFGR1 |= ADC_CFGR1_DMACFG;
        DMA1_Channel1->CCR |= DMA_CCR_CIRC;
        DMA1_Channel1->CCR |= DMA_CCR_HTIE;
    } else {
        ADC1->CFGR1 &= ~ADC_CFGR1_DMACFG;
    }
    
    /* select ADC channel */
    ADC1->CHSELR = (1 << adc_config[line].chan);

    /* enable DMA IRQ */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    /* Enable DMA channel */
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    /* start ADC */
    ADC1->CR |= ADC_CR_ADSTART;
    
    return 0;
}

int adc_sampling_stop(void) {
    /* disable DMA */
    ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

    /* disable DMA channel */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    
    /* disable IRQ */
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    
    /* power off and unlock ADC */
    _disable_adc();
    done();
    
    /* unblock STOP mode */
    pm_unblock(PM_SLEEP);
    
    return 0;
}

void isr_dma1_channel1(void) {
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        /* half-tranfer */
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        
        adc_dma_callback(ADC_DMA_CALLBACK_HALF);
    }
    
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        /* transfer completed */
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        
        if (!(ADC1->CFGR1 & ADC_CFGR1_DMACFG)) {
            adc_sampling_stop();
        }
        
        adc_dma_callback(ADC_DMA_CALLBACK_COMPLETED);
    }
}
