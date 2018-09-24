/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file        ultrasoundrange.c
 * @brief       basic driver for Ultrasonic Rangefinder sensor
 * @author      Dmitry Golik [info@unwds.com]
 */


#include "ultrasoundrange.h"
#include "periph/gpio.h"

#include "xtimer.h"
// #include "periph/timer.h"
#include "periph/pwm.h"
#include "random.h"
#include "rtctimers-millis.h"

// #include "periph/adc.h" // riot's adc driver is too slow

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

// some math

int isqrt(uint32_t x)
{   // http://www.codecodex.com/wiki/Calculate_an_integer_square_root
    register uint32_t op, res, one;  
  
    op = x;  
    res = 0;  
  
    /* "one" starts at the highest power of four <= than the argument. */  
    one = 1 << 30;  /* second-to-top bit set */  
    while (one > op) one >>= 2;  
  
    while (one != 0) {  
        if (op >= res + one) {  
            op -= res + one;  
            res += one << 1;  // <-- faster than 2 * one  
        }  
        res >>= 1;  
        one >>= 2;  
    }
    return res;  
}

// calculate speed of sound at given temperature
// https://en.wikipedia.org/wiki/Speed_of_sound
int speed_of_sound(int temperature) {
    return UZ_SOUNDSPEED_CONSTANT * isqrt(temperature * 1024) / 32; // 0.02005 * sqrt(temperature)
}

int ultrasoundrange_us_to_mm(ultrasoundrange_t *dev, int us) {
    return us * dev -> half_speed_of_sound / UZ_SOUNDSPEED_DIVISOR;
}
// {return us * 17 / 100;} // speed of sound is 0.34 mm/us, so we multiply microseconds by 0.17 and get millimeters

int ultrasoundrange_mm_to_us(ultrasoundrange_t *dev, int mm) {
    return mm * UZ_SOUNDSPEED_DIVISOR / dev -> half_speed_of_sound;
}
// {return mm * 100 / 17;} // speed of sound is 0.34 mm/us, so we divide millimeters by 0.17 and get microseconds

int adc_measure_temperature(void);

int calibrate_sound_speed(ultrasoundrange_t *dev, bool measure_temperature) {
    int t = UZ_DEFAULT_TEMPERATURE;
    if (measure_temperature)
        t = adc_measure_temperature();
    dev -> temperature = t;
    int s = speed_of_sound(t) / 2;
    dev -> half_speed_of_sound = s;
    DEBUG("half_speed_of_sound: %d / %d mm/s\n", s, UZ_SOUNDSPEED_DIVISOR);
    return s;
}

void ultrasoundrange_reset(ultrasoundrange_t *dev) {
    // reset all parameters to defaults
    dev -> transmit_pulses = UZ_DEFAULT_TRANSMIT_PULSES;
    dev -> silencing_pulses = UZ_DEFAULT_SILENCING_PULSES;
    dev -> period_us = UZ_DEFAULT_PERIOD_US;
    dev -> silencing_period_us = UZ_DEFAULT_SILENCING_PERIOD_US;
    dev -> idle_period_us = UZ_DEFAULT_IDLE_PERIOD_US;
    dev -> duty = UZ_DEFAULT_DUTY;
    dev -> duty2 = UZ_DEFAULT_DUTY2;
    dev -> sensitivity = UZ_DEFAULT_SENSITIVITY;
    dev -> sensitivity2 = UZ_DEFAULT_SENSITIVITY2;
    dev -> min_distance = UZ_DEFAULT_MIN_DISTANCE;
    dev -> max_distance = UZ_DEFAULT_MAX_DISTANCE;
    
    dev -> disrupting_pin = UZ_DISRUPTING_PIN;
    dev -> silencing_pin = UZ_SILENCING_PIN;
    dev -> beeping_pin = UZ_BEEPING_PIN;
    dev -> adc_pin = UZ_ADC_PIN;
    dev -> adc_channel = UZ_ADC_CHANNEL;
    
    dev -> pwren_pin = UZ_PWREN_PIN;
    calibrate_sound_speed(dev, false); // calculate speed of sound at default temperature
    // calibrate_sound_speed(dev, true); // calculate speed of sound at measured temperature - temperature measurement works fine on stm32l151ccu6 but crashes on stm32l151cbu6a due to errata
}



/**
 * @brief USOUNDRANGE driver initialization routine
 *
 * @param[out] dev device structure pointer
 * @param[in] param USOUNDRANGE driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int ultrasoundrange_init(ultrasoundrange_t *dev)
{
    ultrasoundrange_reset(dev);
    /* set ADC clock */

    // ADC->CCR &= ~ADC_CCR_ADCPRE;
    // ADC->CCR |= ADC_CLOCK_MEDIUM;
    
    return 0;
}


void ultrasoundrange_turn_off(ultrasoundrange_t *dev)
{   
    gpio_init(dev -> pwren_pin, GPIO_OUT);
    gpio_set(dev -> pwren_pin);
}

void ultrasoundrange_turn_on(ultrasoundrange_t *dev)
{   
    gpio_init(dev -> pwren_pin, GPIO_OUT);
    gpio_clear(dev -> pwren_pin);
    rtctimers_millis_sleep(UZ_PWRON_DELAY_MS);
}


#define CR1_CLEAR_MASK            ((uint32_t)0xFCFFFEFF)
#define CR2_CLEAR_MASK            ((uint32_t)0xC0FFF7FD)

// works fine on stm32l151ccu6 but crashes on stm32l151cbu6a due to errata
int adc_measure_temperature(void){
    // like in cpu/stm32l1/periph/adc.c

    // "The ADC clock which is always the HSI clock"
    if (!(RCC->CR & RCC_CR_HSION)) {
        RCC->CR |= RCC_CR_HSION;
        // Wait for HSI to become ready
        while (!(RCC->CR & RCC_CR_HSION)) {}
    }
    
    periph_clk_en(APB2, RCC_APB2ENR_ADC1EN);
    
    // ADC1 CR1 Configuration
    uint32_t tmpreg1 = ADC1->CR1;
    tmpreg1 &= CR1_CLEAR_MASK;
    // tmpreg1 |= res;
    ADC1->CR1 = tmpreg1;

    // CR2 config
    tmpreg1 = ADC1->CR2;
    tmpreg1 &= CR2_CLEAR_MASK;

    uint32_t align = 0x00000000; // Left alignment of data
    uint32_t edge = 0x00000000;
    uint32_t etrig = 0x03000000; // EXTSEL = 0011: TIM2_CC2 event ???77
    uint32_t continuous_conv_mode = 0; // single conversion mode

    tmpreg1 |= (uint32_t)(align | edge | etrig | ((uint32_t)continuous_conv_mode << 1));
    ADC1->CR2 = tmpreg1;
    
    ADC1->CR2 &= ~ADC_CR2_DELS; // no delay after measurement
    ADC1->SMPR2 &= 0xC0000000; // fastest sampling time at second 10 channels
    // ADC1->SMPR2 |= 0xffffffff; // sampling time 384 cycles
    ADC1->SMPR2 |= 0b101101101101101101101101101; // sampling time 96 cycles
    ADC->CCR &= ~ADC_CCR_ADCPRE; // ADC will be running at HSI clock div 1 (16 MHz)
    // ADC->CCR |= 2; // ADC will be running at HSI clock div 4 (4 MHz)
    ADC1->CR1 &= ~ADC_CR1_JAUTO; // no automatic conversion of injected channels - no effect
    ADC1->CR1 &= ~ADC_CR1_PDI; // 0: The ADC is powered up when waiting for a start event - 1 Msps now!1

    // sampling two channels at once
    ADC1->SQR5 &= 0xC0000000; // resetting register
    ADC1->SQR5 |= ADC_TEMPERATURE_CHANNEL + (ADC_VREF_CHANNEL << 5); // 16th channel is temperature, 17th is Vref
    ADC1->CR1  |= ADC_CR1_SCAN; // works both with and without scan mode with the same sampling frequency
    ADC1->SQR1 &= 0xC0000000; // resetting register
    ADC1->SQR1 |= ((2 - 1) << 20); // length = 2
    ADC->CCR |= ADC_CCR_TSVREFE; // Enable temperature and Vref conversion

    // Enable ADC
    ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;

    // Wait for ADC to become ready
    while ((ADC1->SR & ADC_SR_ADONS) == 0);
    // xtimer_spin(xtimer_ticks_from_usec(20)); // waiting for temperature sensor to stabilize

    ADC1->CR2 |= ADC_CR2_EOCS; // 1: The EOC bit is set at the end of each regular conversion

    int temp = 0;
    int vref = 0;

    for (int i = 0; i < 8; i++) {
        xtimer_spin(xtimer_ticks_from_usec(20)); // waiting for temperature sensor to stabilize - need to wait before each reading!
        // Start conversion on regular channels.
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        // Wait until the end of ADC conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0);
        temp += (int)ADC1->DR; // reading temperature

        ADC1->SR = ~ADC_SR_EOC;
        while ((ADC1->SR & ADC_SR_EOC) == 0); // reading voltage reference
        vref += (int)ADC1->DR;
    }
    // temp = temp / 8;
    // vref = vref / 8;

    ADC->CCR &= ~ADC_CCR_TSVREFE; // Disable temperature and Vref conversion
    // unlock and power off device
    ADC1->CR2 &= ~(ADC_CR2_ADON);
    periph_clk_dis(APB2, RCC_APB2ENR_ADC1EN);

    // int vcal = *ADC_VREFINT_CAL * 8; // vref adc value at 3V
    // int tcal1 = *ADC_TS_CAL1 * 8; // temp adc value at 30C and 3V
    // int tcal2 = *ADC_TS_CAL2 * 8; // temp adc value at 110C and 3V
    int vcal = 1500 * 8; // vref adc value at 3V
    int tcal1 = 630 * 8; // temp adc value at 30C and 3V
    int tcal2 = 830 * 8; // temp adc value at 110C and 3V
    
    // int vdd = 3000 * vcal / vref; // right!1
    // DEBUG("NVref: %d, NVref_cal: %d, Vdd: %d, NTcal1: %d, NTcal2: %d, NTraw: %d, NTadj: %d \n", vref, vcal, vdd, tcal1, tcal2, temp, temp * vcal / vref);
    // temp = (temp * vdd) / 3000; // as in riot - right
    temp = temp * vcal / vref; // same as above
    // temp = (temp * 3000) / vdd; // as in AN3964 - wrong
    
    temp = (80 * (temp - tcal1)) / (tcal2 - tcal1) + 30; // temperature in celsius
    temp += 273; // temperature in kelvins

    DEBUG("Temperature: %d K (%d C)\n", temp, temp - 273);
    return temp;
}


void ultrasoundrange_adc_start(uint8_t pin, uint8_t ch){
    // like in cpu/stm32l1/periph/adc.c
    
    /* "The ADC clock which is always the HSI clock" */
    if (!(RCC->CR & RCC_CR_HSION)) {
        RCC->CR |= RCC_CR_HSION;
        /* Wait for HSI to become ready */
        while (!(RCC->CR & RCC_CR_HSION)) {}
    }
    
    periph_clk_en(APB2, RCC_APB2ENR_ADC1EN);
    
    // ADC1 CR1 Configuration
    uint32_t tmpreg1 = ADC1->CR1;
    tmpreg1 &= CR1_CLEAR_MASK;
    // tmpreg1 |= res;
    ADC1->CR1 = tmpreg1;

    // CR2 config
    tmpreg1 = ADC1->CR2;
    tmpreg1 &= CR2_CLEAR_MASK;

    uint32_t align = 0x00000000; /* Left alignment of data */
    uint32_t edge = 0x00000000;
    uint32_t etrig = 0x03000000; // EXTSEL = 0011: TIM2_CC2 event ???77
    uint32_t continuous_conv_mode = 0; // single conversion mode

    tmpreg1 |= (uint32_t)(align | edge | etrig | ((uint32_t)continuous_conv_mode << 1));
    ADC1->CR2 = tmpreg1;
    
    ADC1->CR2 &= ~ADC_CR2_DELS; // no delay after measurement
    // ADC1->SMPR3 &= ~7; // zeroing SMP0[2:0] - fastest sampling time
    ADC1->SMPR3 &= 0xC0000000; // fastest sampling time at first 10 channels
    ADC1->SMPR2 &= 0xC0000000; // fastest sampling time at second 10 channels
    ADC->CCR &= ~ADC_CCR_ADCPRE; // ADC will be running at HSI clock div 1 (16 MHz)
    ADC1->CR1 &= ~ADC_CR1_JAUTO; // no automatic conversion of injected channels - no effect
    ADC1->CR1 &= ~ADC_CR1_PDI; // 0: The ADC is powered up when waiting for a start event - 1 Msps now!1

    // sampling one channel at once
    ADC1->SQR5 &= 0xC0000000; // resetting register
    ADC1->SQR5 |= ch; // first channel to sample
    ADC1->CR1  |= ADC_CR1_SCAN; // works both with and without scan mode with the same sampling frequency
    ADC1->SQR1 &= 0xC0000000; // resetting register
    ADC1->SQR1 |= ((1 - 1) << 20); // length = 1
    
    // Enable ADC
    ADC1->CR2 |= (uint32_t)ADC_CR2_ADON;
    gpio_init(pin, GPIO_AIN);

    // Wait for ADC to become ready
    while ((ADC1->SR & ADC_SR_ADONS) == 0);

    ADC1->CR2 |= ADC_CR2_EOCS; // 1: The EOC bit is set at the end of each regular conversion
    
}

void ultrasoundrange_adc_stop(void){
    // unlock and power off device
    ADC1->CR2 &= ~(ADC_CR2_ADON);
    periph_clk_dis(APB2, RCC_APB2ENR_ADC1EN);
}

int * adc_read_multi(ultrasoundrange_t *dev, int n, int begin_time){
    
    static int16_t cos[UZ_MAX_READS] = {};
    static int16_t sin[UZ_MAX_READS] = {};
    int reads[4] = {0, 0, 0, 0}; // array of four last adc readings for calculating sine and cosine components
    const int quarter_period = dev->period_us; // length of quarter period in terms of quarters of time units (i. e. 1 us / UZ_QUARTER_PERIOD_DIVISOR)
    if (n > UZ_MAX_READS)
        n = UZ_MAX_READS;
    if (begin_time == 0)
        begin_time = xtimer_now_usec();
    
    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);
    
    int begin_time_precise = begin_time * UZ_QUARTER_PERIOD_DIVISOR;
    int period_n = ((int16_t)(timer_read(XTIMER_DEV) - begin_time)) * UZ_SUBUS_DIVISOR / dev->period_us + 1;
    int next_time_precise = begin_time_precise + period_n * dev->period_us * 4; // should be phase perfect (however measurement error is more than one period)
    int16_t next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0); // waiting for the beginning of measurement
    
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
    next_time_precise += quarter_period;
    next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    int timeout_n = 0;
    for (int i = 0; i < (n * 4); i++) {
        while ((ADC1->SR & ADC_SR_EOC) == 0); // Wait until the end of ADC conversion
        reads[i & 3] = (int)ADC1->DR; // read result
        ADC1->SR = ~ADC_SR_EOC; // int value = (int)ADC1->DR;
        
        if (((int16_t)(timer_read(XTIMER_DEV) - next_time)) >= 0)
            timeout_n += 1; // not enough time
        else
            while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
        
        if ((i & 3) == 3) { // writing values at the end of each period
            cos[i / 4] = reads[0] - reads[2];
            sin[i / 4] = reads[1] - reads[3];
        }
    }
    
    if (timeout_n) {
        DEBUG("[adc_read_multi] Timeout number: %d\n", timeout_n);
    }
    printf("cos_data = array((");
    for (int i = 0; i < n; i++) {
        printf("%d, ", (int)cos[i]);
    }
    printf("));\n");

    printf("sin_data = array((");
    for (int i = 0; i < n; i++) {
        printf("%d, ", (int)sin[i]);
    }
    printf("));\n");
    
    return 0; // TODO: return both cos and sin in parameters and length in return value
}


int adc_all_echoes(ultrasoundrange_t *dev, int16_t begin_time, int max_peaks, uint32_t peak_distances[], uint32_t peak_amps[]){
    // returns number of echo peaks and list of peak times and amplitudes in peak_distances and peak_amps
    
    int peak_n = 0;
    // static uint32_t peak_distances[UZ_MAX_PEAKS];
    // static uint32_t peak_amps[UZ_MAX_PEAKS];
    
    uint32_t peak_amp = UINT32_MAX; // first peak is fictitious so we initially set min_amp equal to peak_amp to eliminate one comparison
    uint32_t min_amp = UINT32_MAX; // first peak is fictitious so we initially set min_amp equal to peak_amp to eliminate one comparison
    uint32_t peak_time = 0;

    const int reads_len = 4 * UZ_AVERAGING_PERIODS;
    int reads[reads_len];
    for (int i = 0; i < reads_len; i++)
        reads[i] = UZ_MAX_ADC / 2;
    int cos = 0; int sin = 0;
    const int quarter_period = dev->period_us; // length of quarter period in terms of quarters of time units (i. e. 1 us / UZ_QUARTER_PERIOD_DIVISOR)
    uint32_t sens = dev -> sensitivity * dev -> sensitivity * UZ_AVERAGING_PERIODS * UZ_AVERAGING_PERIODS;

    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);
    
    int begin_time_precise = begin_time * UZ_QUARTER_PERIOD_DIVISOR;
    int period_n = ((int16_t)(timer_read(XTIMER_DEV) - begin_time)) * UZ_SUBUS_DIVISOR / dev->period_us + 1;
    int next_time_precise = begin_time_precise + period_n * dev->period_us * 4; // should be phase perfect (however measurement error is more than one period)
    int16_t next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    // while((xtimer_now_usec() - begin_time) * UZ_QUARTER_PERIOD_DIVISOR < next_time); // waiting for the beginning of measurement
    while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0); // waiting for the beginning of measurement
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
    next_time_precise += quarter_period;
    next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    int max_readings = (ultrasoundrange_mm_to_us(dev, dev -> max_distance) * UZ_QUARTER_PERIOD_DIVISOR - next_time) / quarter_period; // number of adc readings based on max distance
    
    int timeout_n = 0;
    for (int i = 0; i < max_readings; i++) {
        if (peak_n >= max_peaks) break;
        
        // Wait until the end of ADC conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0);

        // read result
        // reads[i & 3] = (int)ADC1->DR;
        int read = (int)ADC1->DR;
        ADC1->SR = ~ADC_SR_EOC;
        
        if (((int16_t)(timer_read(XTIMER_DEV) - next_time)) >= 0)
            timeout_n += 1; // not enough time
        else
            while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        
        // Start conversion on regular channels.
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        // next_time += quarter_period;
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
   
        
        // rectangular window averaging
        int sign = 1 - (i & 2);
        if (!(i & 1)) {
            cos += (read - reads [i % reads_len]) * sign;
        } else {
            sin += (read - reads [i % reads_len]) * sign;
        }
        reads [i % reads_len] = read;
        
        
        if (!(i & 3)) { // detecting peaks each 1 period cos otherwise calculated amplitude oscillates
            
            uint32_t amp_scaled = cos*cos + sin*sin; // squared filtered amplitude
            
            if (amp_scaled < peak_amp / 4) {
                // valley reached
                if ((peak_amp > sens) && (peak_amp / 4 > min_amp)) {
                    // new valley reached - so we have time to record the last peak
                    // peak is ignored if it is less than 4 times higher than adjacent valleys
                    // if (peak_time > min_time) {
                        peak_amps[peak_n] = peak_amp;
                        peak_distances[peak_n] = peak_time;
                        peak_n++;
                    // }
                    min_amp = amp_scaled;
                } else if (amp_scaled < min_amp) min_amp = amp_scaled; // "else" cos otherwise there will be another comparison in a row
                peak_amp = sens; // new peak will be registered if amplitude exceeds peak_amp (i. e. sens), also valley is reached if and only if peak_amp == sens - so we eliminate one comparison and three assignments
            } else if (amp_scaled > peak_amp) {
                // peak_time = next_time / UZ_QUARTER_PERIOD_DIVISOR;
                peak_time = (uint16_t)(next_time - begin_time);
                peak_amp = amp_scaled;
            }
        }
    }
    
    if ((peak_amp > sens) && (peak_amp / 4 > min_amp)) { // recording the last peak if we had not record it before
        peak_amps[peak_n] = peak_amp;
        peak_distances[peak_n] = peak_time;
        peak_n++;
    }
    
    // calculating distances from times and amplitudes from squared amplitudes
    for (int x = 0; x < peak_n; x++) {
        peak_distances[x] = ultrasoundrange_us_to_mm(dev, peak_distances[x]);
        peak_amps[x] = isqrt(peak_amps[x]) / UZ_AVERAGING_PERIODS;
    }
    
    // filtering peaks by amp*time>sens2 && distance>min_distance criterion
    uint32_t min_distance = dev -> min_distance;
    uint32_t sens2 = dev -> sensitivity2 * UZ_NORMALIZING_DIVISOR;
    int y = 0;
    for (int x = 0; x < peak_n; x++) {
        if ((peak_distances[x] > min_distance) && (peak_amps[x] * peak_distances[x] > sens2)) {
            peak_amps[y] = peak_amps[x];
            peak_distances[y] = peak_distances[x];
            y += 1;
        }
    }
    peak_n = y;

    ultrasoundrange_adc_stop();
    
    if (timeout_n) {
        DEBUG("# [adc_all_echoes] Timeout number: %d of %d\n", timeout_n, max_readings);
    }
    // DEBUG("distances = array((");
    DEBUG("array(((");
    for (int x = 0; x < peak_n; x++) {
        DEBUG("%d, ", (int)peak_distances[x]);
    }
    DEBUG("), ");

    DEBUG("(");
    for (int x = 0; x < peak_n; x++) {
        DEBUG("%d, ", (int)peak_amps[x]);
    }
    DEBUG("), ");
    
    DEBUG("(");
    for (int x = 0; x < peak_n; x++) {
        DEBUG("%d, ", (int)peak_amps[x] * (int)peak_distances[x] / UZ_NORMALIZING_DIVISOR);
    }
    
    DEBUG("))), \n");

    return peak_n;
}

int adc_first_echo(ultrasoundrange_t *dev, int begin_time){
    uint32_t peak_distances[UZ_MAX_PEAKS];
    uint32_t peak_amps[UZ_MAX_PEAKS];
    int peak_n = adc_all_echoes(dev, begin_time, UZ_MAX_PEAKS, peak_distances, peak_amps);
    if (peak_n > 0) {
        return peak_distances[0];
    }
    return -1;
}

static int _ultrasoundrange_transmit(ultrasoundrange_t *dev, int transmit_pulses, int period, int duty, int silencing_pulses, int idle_period, int duty2, int silencing_period) {
    gpio_init(dev->beeping_pin, GPIO_OUT);
    gpio_init(dev->disrupting_pin, GPIO_OUT);
    gpio_clear(dev->disrupting_pin); // disrupting current to op amp
    gpio_init(dev->silencing_pin, GPIO_OUT);
    gpio_clear(dev->silencing_pin); // antiresonance silencing
    
    int T = period; // period in sub-microseconds
    int T1 = duty; // time of first phase in sub-microseconds
    int T2 = T - T1; // time of second phase in sub-microseconds
    int Tidle = idle_period + (T1 - duty2) / 2; // waiting time to make counterphase roughly independent of duty cycles
    if (Tidle < 0) Tidle = 0;

    // transmission!
    
    int begin_time = xtimer_now_usec(); // we are always subtracting this time to prevent integer overflow
    int switch_time_precise = ((int16_t)begin_time) * UZ_SUBUS_DIVISOR;
    int16_t switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
    
    for (int i = 0; i < transmit_pulses; i++){
        gpio_set(dev->beeping_pin);
        switch_time_precise += T1;
        switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
        gpio_clear(dev->beeping_pin);
        switch_time_precise += T2;
        switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
    }
    
    T1 = duty2; // time of first phase in sub-microseconds
    T2 = T - T1; // time of second phase in sub-microseconds
    switch_time_precise += Tidle;
    switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
    
    for (int i = 0; i < silencing_pulses; i++){
        gpio_set(dev->beeping_pin);
        switch_time_precise += T1;
        switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
        gpio_clear(dev->beeping_pin);
        switch_time_precise += T2;
        switch_time = switch_time_precise / UZ_SUBUS_DIVISOR;
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
    }
    
    // enabling current to op amp
    gpio_set(dev->disrupting_pin);
    
    // waiting for silencing_period_us
    // TODO: measure silencing time in millimeters!
    if (silencing_period > 0) {
        gpio_set(dev->silencing_pin); // antiresonance silencing
        switch_time += silencing_period;
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
        gpio_clear(dev->silencing_pin); // end of antiresonance silencing - rc filter should slowly decrease gate voltage
    } else {
        switch_time += 2; // waiting for C2 to charge
        while(((int16_t)(timer_read(XTIMER_DEV) - switch_time)) < 0);
    }
    
    // gpio_init(dev->silencing_pin, GPIO_AIN); // this generates much noise
    gpio_init(dev->beeping_pin, GPIO_AIN); // this generates less noise
    gpio_init(dev->disrupting_pin, GPIO_AIN); // for filtering noise from processor
    
    return begin_time + (period * (transmit_pulses + silencing_pulses) + Tidle + duty2 / 2 - period / 2) / UZ_SUBUS_DIVISOR; // time of end of counterphase silencing
}


/*static*/ int ultrasoundrange_transmit(ultrasoundrange_t *dev) {
    return _ultrasoundrange_transmit(dev, dev->transmit_pulses, dev->period_us, dev->duty, dev->silencing_pulses, dev->idle_period_us, dev->duty2, dev->silencing_period_us);
}



int ultrasoundrange_measure_decay(ultrasoundrange_t *dev, int period, int16_t begin_time, int max_time, bool silencing, bool verbose) {
    
    if (silencing) {
        gpio_init(dev->silencing_pin, GPIO_OUT);
        gpio_set(dev->silencing_pin); // antiresonance silencing
    }
    
    // measurement
    
    const int quarter_period = period; // length of quarter period in terms of quarters of time units (i. e. 1 us / UZ_QUARTER_PERIOD_DIVISOR)
    // int next_time = 0;
    
    int begin_time_precise = begin_time * UZ_QUARTER_PERIOD_DIVISOR;
    int period_n = ((int16_t)(timer_read(XTIMER_DEV) - begin_time)) * UZ_SUBUS_DIVISOR / dev->period_us + 1;
    int next_time_precise = begin_time_precise + period_n * dev->period_us * 4; // should be phase perfect (however measurement error is more than one period)
    int16_t next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    int max_readings = max_time * UZ_QUARTER_PERIOD_DIVISOR / quarter_period; // number of adc readings based on max time
    // int reads[4] = {UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC}; // array of four last adc readings for calculating sine and cosine components
    const int reads_len = 4 * UZ_AVERAGING_PERIODS;
    int reads[reads_len];
    for (int i = 0; i < reads_len; i++)
        reads[i] = UZ_MAX_ADC / 2;
    int cos = 0; int sin = 0;
    uint32_t amp2 = 0;
    
    uint32_t high_sens = UZ_MAX_ADC * UZ_MAX_ADC * UZ_AVERAGING_PERIODS * UZ_AVERAGING_PERIODS / 4;
    uint32_t low_sens = high_sens / 512 * 69; // 69 / 512 is approximately exp(-2), i. e. exp(-1) ** 2
    uint32_t sens = dev -> sensitivity * dev -> sensitivity * UZ_AVERAGING_PERIODS * UZ_AVERAGING_PERIODS;
    uint32_t sens2 = (dev -> sensitivity2 * UZ_NORMALIZING_DIVISOR * UZ_AVERAGING_PERIODS) >> 8;
    sens2 = sens2 * sens2;
    
    int high_amp_time = 0;
    int low_amp_time = 0;
    int sens_amp_time = 0;
    bool low_amp_reached = false;
    
    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);
    
    while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0); // waiting for the beginning of measurement
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
    next_time_precise += quarter_period;
    next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    for (int i = 0; (i < 64) && (amp2 <= high_sens); i++) { // preparing filtered values
    // for (int i = 0; i < 256; i++) { // preparing filtered values - 256 quarter periods is about 1.6 ms
        // Wait until the end of ADC conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0);
        // read result
        // reads[i & 3] = (int)ADC1->DR;
        int read = (int)ADC1->DR;
        
        ADC1->SR = ~ADC_SR_EOC;
        // while((xtimer_now_usec() - begin_time) * UZ_QUARTER_PERIOD_DIVISOR < next_time); // waiting for about 1/4 period
        while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        // Start conversion on regular channels.
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        // next_time += quarter_period;
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
        
        // rectangular window averaging
        int sign = 1 - (i & 2);
        if (!(i & 1)) {
            cos += (read - reads [i % reads_len]) * sign;
        } else {
            sin += (read - reads [i % reads_len]) * sign;
        }
        reads [i % reads_len] = read;
        
        amp2 = cos*cos + sin*sin;
    }
    
    int timeout_n = 0;
    for (int i = 0; i < max_readings; i++) {
        while ((ADC1->SR & ADC_SR_EOC) == 0); // Wait until the end of ADC conversion
        // reads[i & 3] = (int)ADC1->DR; // read result
        int read = (int)ADC1->DR;
        
        ADC1->SR = ~ADC_SR_EOC;
        
        if (((int16_t)(timer_read(XTIMER_DEV) - next_time)) >= 0)
            timeout_n += 1; // not enough time
        else
            while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
        // next_time += quarter_period;
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
        
        // rectangular window averaging
        int sign = 1 - (i & 2);
        if (!(i & 1)) {
            cos += (read - reads [i % reads_len]) * sign;
        } else {
            sin += (read - reads [i % reads_len]) * sign;
        }
        reads [i % reads_len] = read;
        
        amp2 = cos*cos + sin*sin;
        
        if (amp2 > high_sens) {
            high_amp_time = next_time_precise;
        }
        if (amp2 > low_sens) {
            low_amp_time = next_time_precise;
        } else {
            low_amp_reached = true;
        }
        // if (amp2 > sens) {
        if ((amp2 > sens) && ((amp2 >> 8) * ((next_time * next_time) >> 8) > sens2)) {
            sens_amp_time = next_time_precise;
        }
        if (((next_time_precise - low_amp_time) > 64 * quarter_period) && ((next_time_precise - sens_amp_time) > 64 * quarter_period))
            break;
    }
    ultrasoundrange_adc_stop();
    
    if (silencing) {
        gpio_clear(dev->silencing_pin); // end of antiresonance silencing
    }
    
    low_amp_time = (low_amp_time - begin_time_precise) / 4; // times are now in terms of time units (i. e. 1 us / UZ_SUBUS_DIVISOR)
    high_amp_time = (high_amp_time - begin_time_precise) / 4;
    sens_amp_time = (sens_amp_time - begin_time_precise) / 4;
    
    if (timeout_n) {
        DEBUG("[ultrasoundrange_measure_decay] Timeout number: %d of %d\n", timeout_n, max_readings);
    }
    if (verbose) {
        int decay_time = (low_amp_time - high_amp_time) / UZ_SUBUS_DIVISOR;
        int decay_periods4 = (low_amp_time - high_amp_time) * 4 / period;
        if (high_amp_time == 0) {
            decay_time = 0; // if amplitude does not reach desired value
            decay_periods4 = 0;
        }
        printf("Decay parameters ");
        if (silencing) {
            printf(" (with antiresonance silencing)");
        }
        printf(":\n");
        if ((high_amp_time <= 0) && (!silencing))
            printf("    WARNING!! Can not achieve sufficient amplitude! Probably no or improper transceiver connected. All shown results are for debug purposes only!\n");
        if (!low_amp_reached)
            printf("    WARNING!! Ringing lasted till the end of measurement! Probably no or improper transceiver or too much electromagnetic interference. All shown results are for debug purposes only!\n");
        printf("    Characteristic decay time: %d us (%d", decay_time, decay_periods4 / 4);
        if ((decay_periods4 % 4) > 0)
            printf(" %d/4", decay_periods4 % 4);
        printf(" periods)\n");
        printf("    Time of decay to about 0.5 V: %d us (%d periods, %d mm)\n", low_amp_time / UZ_SUBUS_DIVISOR, low_amp_time / period, ultrasoundrange_us_to_mm(dev, low_amp_time / UZ_SUBUS_DIVISOR));
        printf("    Time of decay to sens: %d us (%d periods, %d mm)\n", sens_amp_time / UZ_SUBUS_DIVISOR, sens_amp_time / period, ultrasoundrange_us_to_mm(dev, sens_amp_time / UZ_SUBUS_DIVISOR));
        int Q = decay_periods4 * 804 / 1024; // https://en.wikipedia.org/wiki/Q_factor This means the amplitude falls off to approximately e−π or 4% of its original amplitude. 804 / 256 is about pi
        printf("    Quality factor: %d\n", Q);
        if (Q > 0)
            printf("    Resonance width: +-%d Hz (or +-%d internal units)\n", UZ_SUBUS_DIVISOR * 1000000 / period / Q / 2, period / Q / 2);
    }
    return sens_amp_time / UZ_SUBUS_DIVISOR;
    
}

int ultrasoundrange_measure_period(ultrasoundrange_t *dev) {
    
    // transmission
    int begin_time = _ultrasoundrange_transmit(dev, 10, dev->period_us, dev->period_us / 2 , 0, 0, 0, 0); // only transmission, no silencing
    
    // measurement
    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);
    // int end_time = 0;
    int read = 0;
    int read_n = 0;
    int result = 0;
    int average_read = UZ_HALF_ADC * 128;
    int last_switch_time = xtimer_now_usec() - begin_time;
    for (int i = 0; i < 10; i++) {
        // ignoring first 10 transitions and measuring time of last transition
        // while ((read < UZ_HALF_ADC * 5 / 4) && (read_n < 2000)) {
        while ((read < average_read / 128 + UZ_HALF_ADC / 8) && (read_n < 2000)) {
            // Start conversion on regular channels.
            ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
            // Wait until the end of ADC conversion
            average_read = read - average_read / 128 + average_read; // exponential averaging
            while ((ADC1->SR & ADC_SR_EOC) == 0);
            // read result
            read = (int)ADC1->DR;
            ADC1->SR = ~ADC_SR_EOC;
            read_n++;
        }
        // while ((read > UZ_HALF_ADC * 3 / 4) && (read_n < 2000)) {
        while ((read > average_read / 128 - UZ_HALF_ADC / 8) && (read_n < 2000)) {
            // Start conversion on regular channels.
            ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
            // Wait until the end of ADC conversion
            average_read = read - average_read / 128 + average_read; // exponential averaging
            while ((ADC1->SR & ADC_SR_EOC) == 0);
            // read result
            read = (int)ADC1->DR;
            ADC1->SR = ~ADC_SR_EOC;
            read_n++;
        }
        // average_read = read - average_read / 128 + average_read; // exponential averaging
    }
    
    begin_time = xtimer_now_usec();
    last_switch_time = 0;
    int next_time = 0;
    int max_readings = 100; // number of periods to listen
    for (int i = 0; i < max_readings; i++) {
        while (read < average_read / 128 + UZ_HALF_ADC / 8) {
            // Start conversion on regular channels.
            ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
            // Wait until the end of ADC conversion
            next_time = xtimer_now_usec() - begin_time;
            if (next_time > last_switch_time + 24) {
                // stop if silence is lasting more than maximum supported period cos otherwise we may overestimate the period
                max_readings = i;
                break;
            }
            average_read = read - average_read / 128 + average_read; // exponential averaging
            while ((ADC1->SR & ADC_SR_EOC) == 0);
            // read result
            read = (int)ADC1->DR;
            ADC1->SR = ~ADC_SR_EOC;
            read_n++;
        }
        while (read > average_read / 128 - UZ_HALF_ADC / 8) {
            // Start conversion on regular channels.
            ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
            // Wait until the end of ADC conversion
            next_time = xtimer_now_usec() - begin_time;
            if (next_time > last_switch_time + 48) {
                max_readings = i;
                break;
            }
            average_read = read - average_read / 128 + average_read; // exponential averaging
            while ((ADC1->SR & ADC_SR_EOC) == 0);
            // read result
            read = (int)ADC1->DR;
            ADC1->SR = ~ADC_SR_EOC;
            read_n++;
        }
        if (i < max_readings) {
            last_switch_time = next_time;
            // result = last_switch_time * UZ_SUBUS_DIVISOR / (i + 1);
        }
    }
    result = last_switch_time * UZ_SUBUS_DIVISOR / max_readings;
    ultrasoundrange_adc_stop();
    printf("Period measurement:\n");
    printf("    Measured oscillation period: %d (%d.%02d us)\n", result, result / UZ_SUBUS_DIVISOR, (result * 100 / UZ_SUBUS_DIVISOR) % 100);
    printf("    Frequency: %d Hz\n", UZ_SUBUS_DIVISOR * 1000000 / result);
    DEBUG ("    Periods: %d\n", max_readings);
    DEBUG ("    Reads: %d\n", read_n);
    // DEBUG("    Average reading: %d\n", average_read / 128);
    return result;
    
}


int ultrasoundrange_measure_ringing(ultrasoundrange_t *dev, int idle_period, int duty2, int max_time, int * last_cos, int * last_sin) {
    
    // transmission
    // int begin_time = _ultrasoundrange_transmit(dev, dev->transmit_pulses, dev->period_us, dev->duty, dev->silencing_pulses, idle_period, duty2, 0); // no antiresonance silencing
    int begin_time = _ultrasoundrange_transmit(dev, dev->transmit_pulses, dev->period_us, dev->duty, dev->silencing_pulses, idle_period, duty2, 10); // beginning antiresonance silencing
    
    gpio_set(dev->silencing_pin); // continuing antiresonance silencing
    
    // measurement
    
    // const int quarter_period = period / 4;
    const int quarter_period = dev->period_us; // length of quarter period in terms of quarters of time units (i. e. 1 us / UZ_QUARTER_PERIOD_DIVISOR)
    uint32_t next_time = 0;
    int max_readings = max_time * UZ_QUARTER_PERIOD_DIVISOR / quarter_period; // number of adc readings based on max time
    int reads[4] = {UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC}; // array of four last adc readings for calculating sine and cosine components
    int cos = 0; int sin = 0;
    
    for (int i = 0; i < max_readings - 4; i++) { // waiting for the right time
        while((xtimer_now_usec() - begin_time) * UZ_QUARTER_PERIOD_DIVISOR < next_time);
        next_time += quarter_period;
    }
    
    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);

    // Start conversion on regular channels.
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    
    for (int i = 0; i < 4 * UZ_AVERAGING_PERIODS; i++) { // averaging over 8 periods
        while ((ADC1->SR & ADC_SR_EOC) == 0); // Wait until the end of ADC conversion
        reads[i & 3] = (int)ADC1->DR; // read result
        ADC1->SR = ~ADC_SR_EOC;
        while((xtimer_now_usec() - begin_time) * UZ_QUARTER_PERIOD_DIVISOR < next_time); // waiting for about 1/4 period
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART; // Start conversion on regular channels.
        next_time += quarter_period;
        
        if ((i & 3) == 3) {
            cos += (reads[0] - reads[2]);
            sin += (reads[1] - reads[3]);
        }
    }
    
    ultrasoundrange_adc_stop();
    
    gpio_clear(dev->silencing_pin); // end of antiresonance silencing - rc filter should slowly decrease gate voltage
    
    *last_cos = cos;
    *last_sin = sin;
    return 0;
}


int derivative_projection(int df, int dx, int dy, int * a, int * b) {
    // calculates minimal coefficients of linear function ax + by that gives increment df if arguments incremented by dx and dy
    // given zero increment leaves a and b unchanged and returns 1 as result
    int dr2 = dx * dx + dy * dy;
    if (dr2 == 0) {
        return 1;
    }
    *a = df * dx / dr2;
    *b = df * dy / dr2;
    return 0;
}

int solve2(int64_t a, int64_t b, int64_t p, int64_t c, int64_t d, int64_t q, int * x, int * y) {
    // solves the system of two linear equations
    // returns answer in x and y
    // if it can not find exact solution it returns 1 as result
    // int64_t needed cos numbers are too large
    int64_t det = b * c - a * d;
    if (det == 0) {
        return 1;
    }
    *x = (d * p - b * q) / det;
    *y = (a * q - c * p) / det;
    // return x, y
    return 0;
}

int calc_val(int a, int b, int p, int x, int y) {
    // calculates value of linear function at given point
    return a * x + b * y + p;
}

int calc_dif(int a, int b, int dx, int dy) {
    // calculates difference between values of linear function at given increment
    // same as calc_val(a, b, 0, dx, dy)
    return a * dx + b * dy;
}

int correct_coefs(int * a, int * b, int * p, int f, int x, int y, int df, int dx, int dy, int tolerance, int alpha) {
    // changes coefficients of linear function to better correspond with experimental point f, x, y, df, dx, dy
    // returns coefficients in parameters a, b, p
    // given zero increment leaves a and b unchanged and returns 1 as result
    // alpha is learning constant * 128
    int old_f = calc_val(*a, *b, *p, x, y);
    int df0 = calc_dif(*a, *b, dx, dy);
    int da = 0; int db = 0;
    int result = derivative_projection(df - df0, dx, dy, &da, &db);
    if (result) { // in case of zero increment we do not change anything
        return result;
    }
    
    int dr = isqrt(dx * dx + dy * dy);
    int alpha_1 = alpha;
    if (dr < tolerance)
        alpha_1 = alpha * dr / tolerance; // if coordinate difference is too small then noise will have too much impact on derivative estimation so we decrease learning rate if so
    
    *a += da * alpha_1 / 128;
    *b += db * alpha_1 / 128;
    
    int old_new_p = old_f - calc_val(*a, *b, 0, x, y); // such p that new_f(x, y) = old_f(x, y)
    int new_p = f - calc_val(*a, *b, 0, x, y); // such p that new_f(x, y) = experimental_f(x, y)
    int dp = new_p - old_new_p;
    *p = old_new_p + alpha * dp / 128;
    
    return result;
}

int one_step(ultrasoundrange_t *dev, int * a, int * b, int * p, int * c, int * d, int * q, int * cos0, int * sin0, int x, int y, int dx, int dy, int max_time, int tolerance, int alpha) {
    // alpha is learning constant * 128
    int cos1 = 0; int sin1 = 0;
    // xtimer_spin(xtimer_ticks_from_usec(500));
    rtctimers_millis_sleep(1);
    ultrasoundrange_measure_ringing(dev, x, y, max_time, &cos1, &sin1);
    int result = correct_coefs(a, b, p, cos1, x, y, cos1 - *cos0, dx, dy, tolerance, alpha);
    result += correct_coefs(c, d, q, sin1, x, y, sin1 - *sin0, dx, dy, tolerance, alpha);
    *cos0 = cos1;
    *sin0 = sin1;
    return result;
}

int random_uint16(void) {
    int r = random_uint32();
    return (r + r / 65536) & 65535;
}

int add_random(int * x, int max_dx, int min_x, int max_x) {
    // adds random component to variable cos if distance between points is small, derivative estimation suffers large disturbances due to noise
    // if we are outside target region then set random point within this region
    int min = *x - max_dx;
    int max = *x + max_dx;
    if (min_x > min) min = min_x;
    if (max_x < max) max = max_x;
    int result = 0;
    if (min > max) {
        min = min_x;
        max = max_x;
        result = 1;
    }
    *x = (max - min) * random_uint16() / 65536 + min;
    return result;
}

bool inside(int y, int min_y, int max_y) {return ((y >= min_y) && (y <= max_y));}

int ultrasoundrange_optimize_counterphase(ultrasoundrange_t *dev, int * idle, int * duty2, int point_n, int min_x, int min_y, int max_x, int max_y, int max_time, int tolerance, int alpha, bool verbose) {
    // optimizes idle_period and duty2 (called x and y here)
    // returns number of collected points and idle_period and duty2 in parameters
    // alpha is learning constant * 128
    int x_avg = 0; int y_avg = 0; int avg_n = 0;
    int x0 = min_x;  int y0 = min_y; // coordinates of previous experimental data point
    add_random(&x0, max_x - min_x, min_x, max_x);
    add_random(&y0, max_y - min_y, min_y, max_y);
    int cos0 = 0; int sin0 = 0; 
    ultrasoundrange_measure_ringing(dev, x0, y0, max_time, &cos0, &sin0); // gathering first data point
    int a = 0; int b = 0; int p = 0; // coefficients of linear approximation cos = ax + by + p
    int c = 0; int d = 0; int q = 0; // coefficients of linear approximation sin = cx + dy + q
    int x = 0; int y = 0; // coordinates of current point
    int xs0 = 0; int ys0 = 0; // coordinates of previous approximation of solution
    int no_solution = 1;
    if (verbose) printf("data = array((");
    int i = 0;
    for (i = 0; (avg_n < point_n) && (i < point_n * 4); i++) {
        if (no_solution) {
            add_random(&x, max_x - min_x, min_x, max_x);
            add_random(&y, max_y - min_y, min_y, max_y);
        } else {
            add_random(&x, tolerance, min_x, max_x);
            add_random(&y, tolerance, min_y, max_y);
        }
        no_solution = one_step(dev, &a, &b, &p, &c, &d, &q, &cos0, &sin0, x, y, x - x0, y - y0, max_time, tolerance, alpha);
        x0 = x; y0 = y;
        no_solution += solve2(a, b, p, c, d, q, &x, &y);
        // printf("(%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d), ", no_solution, x, y, x - x0, y - y0, cos0, sin0, a, b, p, c, d, q);
        if (verbose) printf("(%d, %d, %d, %d), ", x, y, cos0, sin0);
        if (!no_solution && (i > 10) && (inside(x, min_x, max_x)) && (inside(y, min_y, max_y)) && ((x - xs0) * (x - xs0) + (y - ys0) * (y - ys0) < tolerance * tolerance)) {
            // if we are close enough to apparent minimum then gather statistics
            x_avg += x; y_avg += y; avg_n += 1;
        }
        xs0 = x; ys0 = y;
    }
    if (verbose) printf("))\n");
    *idle = x_avg / avg_n;
    *duty2 = y_avg / avg_n;
    if (verbose) printf("Optimum estimation: idle %d, duty2 %d based on %d / %d points\n", *idle, *duty2, avg_n, i);
    return avg_n;
}


int ultrasoundrange_measure_noise(ultrasoundrange_t *dev, int period, int max_time) {
    gpio_init(dev->beeping_pin, GPIO_OUT); // slightly less noise in this case (however silencing circuit generates much more noise if its input is in GPIO_AIN)
    gpio_clear(dev->beeping_pin);
    // gpio_init(dev->beeping_pin, GPIO_AIN);
    
    gpio_init(dev->disrupting_pin, GPIO_OUT);
    gpio_set(dev->disrupting_pin); // enabling current to op amp
    xtimer_spin(xtimer_ticks_from_usec(10));
    gpio_init(dev->disrupting_pin, GPIO_AIN); // for filtering noise from processor
    // xtimer_spin(xtimer_ticks_from_usec(10000));
    rtctimers_millis_sleep(10);
    // int begin_time = xtimer_now_usec();
    
    // measurement
    // const int quarter_period = period / 4;
    const int quarter_period = period; // length of quarter period in terms of quarters of time units (i. e. 1 us / UZ_QUARTER_PERIOD_DIVISOR)
    // int next_time = 0;
    int max_readings = max_time * UZ_QUARTER_PERIOD_DIVISOR / quarter_period; // number of adc readings based on max time
    uint64_t avg_raw = 0;
    uint64_t avg_raw2 = 0;
    uint64_t max_raw = 0;
    uint64_t min_raw = UZ_MAX_ADC;
    uint64_t avg_amp2 = 0;
    int max_amp2 = 0;
    
    const int reads_len = 4 * UZ_AVERAGING_PERIODS;
    int reads[reads_len];
    for (int i = 0; i < reads_len; i++)
        reads[i] = UZ_MAX_ADC / 2;
    // int reads[4] = {UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC, UZ_HALF_ADC}; // array of four last adc readings for calculating sine and cosine components
    int cos = 0; int sin = 0;
    
    ultrasoundrange_adc_start(dev->adc_pin, dev->adc_channel);
    
    int next_time_precise = timer_read(XTIMER_DEV) * UZ_QUARTER_PERIOD_DIVISOR; // should be phase perfect (however measurement error is more than one period)
    int16_t next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
    
    // Start conversion on regular channels.
    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    
    for (int i = 0; i < reads_len * 2; i++) { // preparing filtered values
        // Wait until the end of ADC conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0);
        // read result
        int read = (int)ADC1->DR;
        // reads[i & 3] = (int)ADC1->DR;
        ADC1->SR = ~ADC_SR_EOC;
        // while((xtimer_now_usec() - begin_time) * UZ_QUARTER_PERIOD_DIVISOR < next_time); // waiting for about 1/4 period
        while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        // Start conversion on regular channels.
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        // next_time += quarter_period;
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
        
        // rectangular window averaging
        int sign = 1 - (i & 2);
        if (!(i & 1)) {
            cos += (read - reads [i % reads_len]) * sign;
        } else {
            sin += (read - reads [i % reads_len]) * sign;
        }
        reads [i % reads_len] = read;
        
    }
    
    int timeout_n = 0;
    for (int i = 0; i < max_readings; i++) {
        // Wait until the end of ADC conversion
        while ((ADC1->SR & ADC_SR_EOC) == 0);
        // read result
        uint32_t read = (uint32_t)ADC1->DR;
        ADC1->SR = ~ADC_SR_EOC;
        
        if (((int16_t)(timer_read(XTIMER_DEV) - next_time)) >= 0)
            timeout_n += 1; // not enough time
        else
            while(((int16_t)(timer_read(XTIMER_DEV) - next_time)) < 0);
        
        // Start conversion on regular channels.
        ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
        // next_time += quarter_period;
        next_time_precise += quarter_period;
        next_time = next_time_precise / UZ_QUARTER_PERIOD_DIVISOR;
        
        // rectangular window averaging
        int sign = 1 - (i & 2);
        if (!(i & 1)) {
            cos += (read - reads [i % reads_len]) * sign;
        } else {
            sin += (read - reads [i % reads_len]) * sign;
        }
        reads [i % reads_len] = read;
        
        avg_raw += read;
        avg_raw2 += read * read;
        if (read > max_raw) max_raw = read;
        if (read < min_raw) min_raw = read;
        if (!(i & 3)) {
            int amp2 = cos*cos + sin*sin;
            avg_amp2 += amp2;
            if (amp2 > max_amp2) max_amp2 = amp2;
        }
    }
    ultrasoundrange_adc_stop();
    
    int raw_variance = (avg_raw2 - (avg_raw * avg_raw) / max_readings ) / (max_readings - 1);
    avg_raw = avg_raw / max_readings;
    uint64_t max_raw_deviation = max_raw - avg_raw;
    if (avg_raw - min_raw > max_raw_deviation) max_raw_deviation = avg_raw - min_raw;
    
    avg_amp2 = (avg_amp2 / (max_readings / 4)) / UZ_AVERAGING_PERIODS / UZ_AVERAGING_PERIODS;
    max_amp2 = max_amp2 / UZ_AVERAGING_PERIODS / UZ_AVERAGING_PERIODS;
    
    if (timeout_n) {
        DEBUG("[ultrasoundrange_measure_noise] Timeout number: %d of %d\n", timeout_n, max_readings);
    }
    printf("Noise measurement for %d microseconds:\n", max_time);
    printf("    Average raw reading value: %d\n", (int)avg_raw);
    printf("    Minimum raw reading value: %lld\n", min_raw);
    printf("    Maximum raw reading value: %lld\n", max_raw);
    printf("    Maximum reading deviation: %lld\n", max_raw_deviation);
    printf("    Average squared reading deviation (variance): %d\n", raw_variance);
    printf("    Standard deviation: %d\n", isqrt(raw_variance));
    printf("    Average filtered amplitude: %d\n", isqrt(avg_amp2));
    printf("    Maximum filtered amplitude: %d\n", isqrt(max_amp2));
    return max_amp2;
}

/**
 * @brief Gets USOUNDRANGE measure.
 *
 * @param[in] dev pointer to the initialized USOUNDRANGE device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * ultrasoundrange_measure_t measure;
 *
 * ultrasoundrange_init(usoundrange);
 * ultrasoundrange_measure(usoundrange, &measure)
 */

uint32_t ultrasoundrange_measure(ultrasoundrange_t *dev, ultrasoundrange_measure_t *measure)
{
    assert(dev != NULL);

    // adc_measure_temperature();
    // calibrate_sound_speed(dev, true); // works fine on stm32l151ccu6 but crashes on stm32l151cbu6a due to errata

    /*
    int t = 0;
    DEBUG("data = (");    
    for (int i = 0; i < 10; i++) {
        int begin_time = ultrasoundrange_transmit(dev);
        // int t = adc_first_echo(dev, begin_time);
        t = adc_all_echoes(dev, begin_time);
        // xtimer_spin(xtimer_ticks_from_usec(100000));
        rtctimers_millis_sleep(100);
    }
    DEBUG(");\n");
    */

    int begin_time = ultrasoundrange_transmit(dev);
    int result = adc_first_echo(dev, begin_time);
    
    if (result > 0)
        measure->range = result;
    else
        measure->range = -1;
    return 0;
}


int * ultrasoundrange_measure_analog(ultrasoundrange_t *dev, int n)
{
    assert(dev != NULL);
    
    if (n > UZ_MAX_READS) n = UZ_MAX_READS;

    int begin_time = ultrasoundrange_transmit(dev);
    // xtimer_spin(xtimer_ticks_from_usec(1000)); // waiting for transmitter to relax
    return adc_read_multi(dev, n, begin_time); // 5 is number of adc channel (dio 5)
}

int ultrasoundrange_test(ultrasoundrange_t *dev) {
    
    int period = ultrasoundrange_measure_period(dev);
    if ((period < 400) || (period > 1600)) {
        printf("WARNING: Incorrect period (%d)  measured. Maybe there is no or damaged ultrasonic transceiver. Proceeding with default period (%d)\n", period, UZ_DEFAULT_PERIOD_US);
        period = UZ_DEFAULT_PERIOD_US;
    }
    // xtimer_spin(xtimer_ticks_from_usec(100000)); // wait till all echoes have gone
    rtctimers_millis_sleep(100); // wait till all echoes have gone
    ultrasoundrange_measure_noise(dev, period, 100000);

    // decay measurement with and without antiresonance silencing
    printf("Decay measurement with no silencing:\n");
    // xtimer_spin(xtimer_ticks_from_usec(10000));
    rtctimers_millis_sleep(10);
    int begin_time = _ultrasoundrange_transmit(dev, 10, period, period / 2, 0, 0, 0, 0); // only transmission, no silencing
    ultrasoundrange_measure_decay(dev, period, begin_time, 10000, false, true);
    printf("Decay measurement with antiresonance silencing:\n");
    // xtimer_spin(xtimer_ticks_from_usec(10000));
    rtctimers_millis_sleep(10);
    begin_time = _ultrasoundrange_transmit(dev, 10, period, period / 2, 0, 0, 0, 0);
    ultrasoundrange_measure_decay(dev, period, begin_time, 10000, true, true);
    printf("Decay measurement with both counterphase and antiresonance silencing:\n");
    // xtimer_spin(xtimer_ticks_from_usec(10000));
    rtctimers_millis_sleep(10);
    begin_time = _ultrasoundrange_transmit(dev, dev->transmit_pulses, dev->period_us, dev->duty, dev->silencing_pulses, dev->idle_period_us, dev->duty2, 5);
    ultrasoundrange_measure_decay(dev, period, begin_time, 10000, true, true);
    
    return period;
}

int ultrasoundrange_tune(ultrasoundrange_t *dev, int * period, int * idle, int * duty2, int * silencing, int point_n, int max_time, int tolerance, int alpha) {
    
    int old_period = dev -> period_us;
    int old_duty = dev -> duty;
    *period = ultrasoundrange_measure_period(dev);
    if ((*period < 400) || (*period > 1600)) {
        printf("WARNING: Incorrect period (%d)  measured. Maybe there is no or damaged ultrasonic transceiver. Proceeding with default period (%d)\n", *period, UZ_DEFAULT_PERIOD_US);
        *period = UZ_DEFAULT_PERIOD_US;
    }
    dev -> period_us = *period;
    dev ->      duty = *period / 2;
    
    int p = *period;
    int confidence = ultrasoundrange_optimize_counterphase(dev, idle, duty2, point_n, p/4, p/8, p*3/4, p*5/8, max_time, tolerance, alpha, false);
    
    // measuring silencing time needed to decay
    *silencing = 0;
    int silencing_stat[10];
    for (int i = 0; i < 10; i++) {
        // xtimer_spin(xtimer_ticks_from_usec(10000));
        rtctimers_millis_sleep(10);
        int begin_time = _ultrasoundrange_transmit(dev, dev->transmit_pulses, dev->period_us, dev->duty, dev->silencing_pulses, dev->idle_period_us, dev->duty2, 5);
        int s = ultrasoundrange_measure_decay(dev, *period, begin_time, 10000, true, false);
        if (s > *silencing) *silencing = s;
        silencing_stat[i] = s;
    }
    
    /*
    // taking second largest time of decay
    for (int i = 0; i < 10; i++){
        if (silencing_stat[i] == *silencing) {
            silencing_stat[i] = 0;
            break;
        }
    }
    *silencing = 0;
    for (int i = 0; i < 10; i++)
        if (silencing_stat[i] > *silencing) *silencing = silencing_stat[i];
    */
    
    // taking average time of decay
    *silencing = 0;
    for (int i = 0; i < 10; i++)
        *silencing += silencing_stat[i];
    *silencing = *silencing / 10;
    
    if (confidence >= point_n) {
        return 0;
    } else {
        dev -> period_us = old_period;
        dev -> duty = old_duty;
        return 1; // error - no convergence of silencing parameters
    }
}
    
#ifdef __cplusplus
}
#endif
