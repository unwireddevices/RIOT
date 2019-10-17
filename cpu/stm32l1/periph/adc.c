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
#include "periph/pm.h"

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
 * @brief   ADC calibration addresses
 *
 */
#define  ADC_VREFINT_CAL    ((uint16_t *)0x1FF800F8)      /*!< VREFINT calibration data address */
#define  ADC_TS_CAL1        ((uint16_t *)0x1FF800FA)      /*!< Temperature Sensor calibration data address */
#define  ADC_TS_CAL2        ((uint16_t *)0x1FF800FE)      /*!< Temperature Sensor calibration data address */

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
 * @brief   Allocate locks for all three available ADC device
 *
 * All STM32l1 CPU's have single ADC device
 */
static mutex_t lock = MUTEX_INIT;

static bool hsi_enabled = true;

static inline void start(void) {
    /* enable the ADC module */
    ADC1->CR2 |= ADC_CR2_ADON;

    /* Wait for ADC to become ready */
	while (!(ADC1->SR & ADC_SR_ADONS)) {};
}

static inline void stop(void) {
    /* disable the ADC module */
    ADC1->CR2 &= ~ADC_CR2_ADON;

    /* Wait for ADC to become ready */
	while (ADC1->SR & ADC_SR_ADONS) {};
}

static inline void prep(void)
{
    mutex_lock(&lock);
    /* ADC clock is always HSI clock */
    if (!(RCC->CR & RCC_CR_HSION)) {
        while (RCC->CR & RCC_CR_HSIRDY) {}
        hsi_enabled = false;
        RCC->CR |= RCC_CR_HSION;
        /* Wait for HSI to become ready */
        while (!(RCC->CR & RCC_CR_HSIRDY)) {}
    }

    periph_clk_en(APB2, RCC_APB2ENR_ADC1EN);
}

static inline void done(void)
{   
    stop();

    periph_clk_dis(APB2, RCC_APB2ENR_ADC1EN);
    
    if (!hsi_enabled) {
        RCC->CR &= ~RCC_CR_HSION;
    }

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
    
    /* reset ADC */
    RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
    RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;

    /* configure the pin */
    if ((adc_config[line].pin != GPIO_UNDEF))
        gpio_init_analog(adc_config[line].pin);

    /* set ADC clock prescaler */
    ADC->CCR &= ~ADC_CCR_ADCPRE;
    ADC->CCR |= ADC_CLOCK_HIGH;

    /* Set 1 us sample time */
    /* Min 4 us needed for temperature sensor (ADC_IN16) measurements */
    /* Total conversion time is Tsample + 12/Fadc, i.e. 1.75 us with 16 MHz and 9 cycles sampling */
    if (adc_config[line].pin != GPIO_UNDEF) {
        switch (ADC->CCR & ADC_CCR_ADCPRE) {
            case ADC_CLOCK_LOW:
                /* 4 MHz ADC clock -> 4 cycles */
                adc_set_sample_time(ADC_SAMPLE_TIME_4C);
                break;
            case ADC_CLOCK_MEDIUM:
                /* 8 MHz ADC clock -> 4 cycles */
                adc_set_sample_time(ADC_SAMPLE_TIME_9C);
                break;
            default:
                /* 16 MHz ADC clock -> 16 cycles */
                adc_set_sample_time(ADC_SAMPLE_TIME_16C);
        }
    } else {
        /* 96 cycles for 4 us @ 16 MHz minimum sampling time for internal channels */
        adc_set_sample_time(ADC_SAMPLE_TIME_96C);
    }
    
    /* don't turn off during idle phase*/
    ADC1->CR1 &= ~ADC_CR1_PDI;
    
    /* check if this channel is an internal ADC channel, if so
     * enable the internal temperature and Vref */
    if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL || adc_config[line].chan == ADC_VREF_CHANNEL) {
        ADC->CCR |= ADC_CCR_TSVREFE;
        while ((PWR->CSR & PWR_CSR_VREFINTRDYF) == 0);
    }

    /* free the device again */
    done();

    return 0;
}

#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)

int adc_sample(adc_t line,  adc_res_t res)
{
    int sample;

    /* check if resolution is applicable */
    if ( (res != ADC_RES_6BIT) &&
         (res != ADC_RES_8BIT) &&
         (res != ADC_RES_10BIT) &&
         (res != ADC_RES_12BIT)) {
        return -1;
    }

    /* lock and power on the ADC device  */
    prep();

    /* set resolution, conversion channel and single read */
    ADC1->CR1 |= res & ADC_CR1_RES;
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR5 = adc_config[line].chan;

    /* wait for regular channel to be ready*/
    while (ADC1->SR & ADC_SR_RCNR) {}
    
    start();

    /* start conversion and wait for results */
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    /* finally read sample and reset the STRT bit in the status register */
    sample = (int)ADC1->DR;
    ADC1 -> SR &= ~ADC_SR_STRT;
    
    int cal_vref, cal_ts1, cal_ts2;
    /* In case of VREF channel calculate and return actual VDD, not Vref */
	if (adc_config[line].chan == ADC_VREF_CHANNEL) {
        if (cpu_status.category < 3) {
            /* low-end devices doesn't provide calibration values, see errata */
            cal_vref = 1672;
        } else {
            cal_vref = *(uint16_t *)ADC_VREFINT_CAL;
        }
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
        
        sample = (3000 * cal_vref) / sample;
	}
    
    /* in case of temperature channel sample VDD too */
    int sample_vref = 0;
    if (adc_config[line].chan == ADC_TEMPERATURE_CHANNEL) {
        ADC1->SQR5 = ADC_VREF_CHANNEL;
        /* wait for regulat channel to be ready*/
        while (!(ADC1->SR & ADC_SR_RCNR)) {}
        
        /* start conversion and wait for results */
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC)) {}
        
        sample_vref = (int)ADC1->DR;
        ADC1 -> SR &= ~ADC_SR_STRT;
        
        /* calibrate temperature data */
        if (cpu_status.category < 3) {
            /* low-end devices doesn't provide calibration values, see errata */
            /* values according to STM32L151x6/8/B-A datasheet, tables 17 and 59 */
            cal_ts1   = 680;
            cal_ts2   = 856;
            cal_vref  = 1671;
        } else {
            cal_ts1   = *(uint16_t *)ADC_TS_CAL1;
            cal_ts2   = *(uint16_t *)ADC_TS_CAL2;
            cal_vref  = *(uint16_t *)ADC_VREFINT_CAL;
        }
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
        sample = 30 + (80*(sample - cal_ts1))/(cal_ts2 - cal_ts1);
    }
       
    /* Disable temperature and Vref conversion */
	ADC->CCR &= ~ADC_CCR_TSVREFE;
    
    /* power off and unlock device again */
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
    
    adc_dma_callback = adc_cb;

    /* lock and power on the ADC device  */
    prep();

    /* reset DMA bit */
    ADC1->CR2 &= ~ADC_CR2_DMA;
    
    /* enable DMA clock */
    periph_clk_en(AHB, RCC_AHBENR_DMA1EN);
    /* disable DMA channel */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    
    /* set resolution */
    ADC1->CR1 |= res & ADC_CR1_RES;
    
    /* set trigger event */
    ADC1->CR2 &= ~ADC_CR2_EXTSEL;
    ADC1->CR2 |= ((uint32_t)adc_config[line].trigger << 24);
    ADC1->CR2 &= ~ADC_CR2_EXTEN;
    ADC1->CR2 |= ADC_CR2_EXTEN_0;
    
    /* enable DMA */
    ADC1->CR2 |= ADC_CR2_DMA;
    
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
    ADC1->CR1 &= ~ADC_CR1_EOCIE;
    
    if (mode == ADC_CONTINUOUS_CIRCULAR) {
        ADC1->CR2 |= ADC_CR2_DDS;
        DMA1_Channel1->CCR |= DMA_CCR_CIRC;
        DMA1_Channel1->CCR |= DMA_CCR_HTIE;
    } else {
        ADC1->CR2 &= ~ADC_CR2_DDS;
    }
    
    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR5 = adc_config[line].chan;

    /* wait for regular channel to be ready*/
    while (ADC1->SR & ADC_SR_RCNR) {}
    
    /* enable DMA IRQ */
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    
    /* Enable DMA channel */
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    start();
    
    /* block STOP mode */
    pm_block(PM_SLEEP);
    
    return 0;
}

int adc_sampling_stop(void) {
    /* reset DMA bit */
    ADC1->CR2 &= ~ADC_CR2_DMA;

    /* disable DMA channel */
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    
    /* disable IRQ */
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    
    /* power off and unlock ADC */
    done();
    
    /* unblock STOP mode */
    pm_unblock(PM_SLEEP);
    
    return 0;
}

void isr_dma1_ch1(void) {
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        /* half-tranfer */
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        
        adc_dma_callback(ADC_DMA_CALLBACK_HALF);
    }
    
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        /* transfer completed */
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        
        if (!(ADC1->CR2 & ADC_CR2_DDS)) {
            adc_sampling_stop();
        }
        
        adc_dma_callback(ADC_DMA_CALLBACK_COMPLETED);
    }
}
