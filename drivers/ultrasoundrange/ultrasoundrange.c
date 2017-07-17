/* Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @brief       basic driver for Ultrasound Rangefined sensor
 * @authoh      Dmitry Golik [info@unwds.com]
 */


#include "ultrasoundrange.h"
#include "periph/gpio.h"

#include "xtimer.h"
#include "periph/timer.h"
#include "periph/pwm.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif


#define TEST_PERIOD 10000

// both channels - as in riot
#define CCMR_LEFT           (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2)
#define CCMR_RIGHT          (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);

// channel 1 only
// #define CCMR_LEFT           (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
// #define CCMR_RIGHT          (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)


// Вычисление тригонометрических функций с использованием
// целочисленных операций (используется формат с фиксированной точкой).

#define _USE_MATH_DEFINES
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Средства преобразования чисел: float_to_fix16, fix16_to_float.

// Преобразование из обычного формата представления чисел
// (с плавающей точкой или целого) в формат с фиксированной
// точкой: 16 бит дробная и 16 бит целая часть.
#define float_to_fix16(x) (int)((x)*0x10000)
// #define float_to_fix24(x) (int)((x)*0x1000000)

// Преобразование числа с фиксированной точкой (16 бит в дробной части)
// в число с плавающей точкой.
#define fix16_to_float(x) ((x)/65536.0)

// Определяем тип с фиксированной точкой (16 бит в дробной части).
// Эквивалентен целому со знаком и используется только в целях
// обеспечения выразительности кода.
typedef int fix16;

// Функция быстрого вычисления sin, |x|<=M_PI/4.
fix16 _fast_sin(fix16 x)
{
    // Вычисляется x*x: предкоррекция, целочисленное умножение, посткоррекция.
    fix16 x2=(x>>1)*x>>15; // x2=x**2.
    // fix16 x2 = (x >> 8) * (x >> 8); // x2=x**2.

    // fix16 xn=x2*x>>16;     // xn=x**3.
    fix16 xn=x2 * (x >> 1) >> 15;     // xn=x**3.
    // fix16 xn = (x2 >> 8) * (x2 >> 8);     // xn=x**3.

    fix16 r = x - (xn * float_to_fix16(1/6.0) >> 16);
    // fix16 r = x - ((xn >> 8) * (float_to_fix16(1/6.0) >> 8));

    // xn *= x2; xn >>= 16;       // xn=x**5.
    xn *= (x2 >> 1); xn >>= 15;       // xn=x**5.
    // xn = (xn >> 8) * (x2 >>= 8);       // xn=x**5.

    // r=x-(x**3)/6.0+(x**5)/120.0.
    r += (xn * float_to_fix16(1/120.0) >> 16);
    // r += ((xn >> 8) * (float_to_fix16(1/120.0) >> 8));

    return r;
}

// Функция быстрого вычисления cos, |x|<=M_PI/4.
fix16 _fast_cos(fix16 x)
{
    // Вычисляется x*x: предкоррекция, целочисленное умножение, посткоррекция.
    fix16 x2 = x * (x>>1) >> 15; // x2=x**2.

    // fix16 xn=x2*x2>>16;    // xn=x**4.
    fix16 xn = x2 * (x2 >> 1) >> 15;    // xn=x**4.

    // fix16 r = float_to_fix16(1.0) - (x2 * float_to_fix16(0.5) >> 16) + (xn * float_to_fix16(1/24.0) >> 16);
    fix16 r = float_to_fix16(1.0) - (x2 >> 1) + (xn * float_to_fix16(1/24.0) >> 16);

    // xn*=x2; xn>>=16;       // xn=x**6.
    xn *= (x2 >> 1); xn>>=15;       // xn=x**6.

    // r=1-(x**2)/2.0+(x**4)/24.0-(x**6)/720.0.
    r -= xn * float_to_fix16(1.0/720) >> 16;
    
    return r;
}

/* fix16 _fast_cos(fix16 x) // не работает!!1 ибо надо узнать, как производить операции с 64-битным результатом умножения!
{
    // Вычисляется x*x: предкоррекция, целочисленное умножение, посткоррекция.
    fix16 x2 = x * (x>>1) >> 7; // x2=x**2. // промежуточные значения в формате 8.24, ибо всё равно оно меньше 1

    // fix16 xn=x2*x2>>16;    // xn=x**4.
    fix16 xn = x2 * (x2 >> 1) >> 23;    // xn=x**4.

    fix16 r = float_to_fix24(1.0) - (x2 * float_to_fix24(0.5) >> 24) + (xn * float_to_fix24(1/24.0) >> 24);

    // xn*=x2; xn>>=16;       // xn=x**6.
    xn *= (x2 >> 1); xn>>=23;       // xn=x**6.

    // r=1-(x**2)/2.0+(x**4)/24.0-(x**6)/720.0.
    r -= xn * float_to_fix24(1.0/720) >> 24;
    
    return r >> 8;
} */


/*fix16 _fast_cos_1(fix16 x){
    if(x < float_to_fix16(0.9)) // 0.9 is magic number at which errors of _fast_cos and _fast_sin is approximately equal
        return _fast_cos(x);
    else
        return _fast_sin(float_to_fix16(M_PI / 2.) - x);
}*/

// Функция вычисления sin от аргумента, заданного индексом фазы.
fix16 fast_sin(unsigned int k)
{
    fix16 (*ftable[])(fix16) ={_fast_sin, _fast_cos};

    // Индексу фазы k соответствует фаза (аргумент) phi=2*M_PI*k/(2**32).
    // phi можно представить в виде:
    // phi=n*M_PI/2+alpha, где n - некоторое целое, а |alpha|<=M_PI/4.

    // Вычисляем n.
    unsigned int n=(k+0x20000000)>>30;

    // Вычисляем соответствующий alpha индекс фазы (целое со знаком).
    int ka=k-(n<<30);  // fix32, 30bits

    // Преобразуем индекс фазы в фазу в формате fix16.
    fix16 alpha=               // fix30, 31bits
        (int)(4096*2*M_PI)*    // 2*M_PI, fix12, 15bits
        (ka>>14);              // fix18, 16bits
    alpha>>=14;                // fix16

    // r=_fast_sin(alpha) или r=_fast_cos(alpha), в зависимости
    // от младшего бита числа n:
    fix16 r=ftable[n&1](alpha);

    // Если бит [1] в n установлен, в качестве результата берём
    // r с противоположным знаком: if(n&2) r=-r;
    int s=(n>>1)&1;
    return (r^(-s))+s;
}

fix16 fast_cos(unsigned int k) {return fast_sin(k + (1 << 30));}


// синусы оканчиваются!

// а теперь начинаются арктангенсы!

// atan(d) ~ d - d**3 / 3. + d**5 / 5. - d**7 / 7. - taylor series - accurate if d < 0.4
// fix16 _atan2_t7(int y, int x) {
//     fix16 d = (y << 16) / x; // y / x in fix16 notation
fix16 _atan_t7(fix16 d) {
    fix16 d2 = d * (d >> 1) >> 15; // d2=d**2.
    fix16 dn = d2 * (d >> 1) >> 15;     // dn=x**3.
    fix16 r = d - (dn * float_to_fix16(1/3.) >> 16);
    dn *= (d2 >> 1); dn >>= 15;       // dn=x**5.
    r  += (dn * float_to_fix16(1/5.) >> 16);
    dn *= (d2 >> 1); dn >>= 15;       // dn=x**7.
    r  -= (dn * float_to_fix16(1/7.) >> 16);
    return r;
}

// atan(d) ~ -0.02458677 + 1.16334442 * x - 0.39347984 * x**2 + 0.04000809 * x**3 - least squares polynomial fit for [0.4 .. 1)
// fix16 _atan2_p4(int y, int x) {
//     fix16 d = (y << 16) / x; // y / x in fix16 notation
fix16 _atan_p4(fix16 d) {
    fix16 dn = (d >> 1) * (d >> 1) >> 14; // dn=d**2.
    fix16 r = float_to_fix16(-0.02458677) + ((d * float_to_fix16(1.16334442 / 4)) >> 14) - ((dn * float_to_fix16(0.39347984 / 2)) >> 15);
    dn = (dn >> 1) * (d >> 1) >> 14;     // dn=d**3.
    r += dn * float_to_fix16(0.04000809) >> 16;
    return r;    
}

// |y| must be less than 32768 (or 0.5 in fix16), x must be > 0, y must be less than y but more than 0
fix16 _atan2(int y, int x) {
    fix16 d = ((y << 15) / x) << 1; // y / x in fix16 notation
    if (d < float_to_fix16(0.4235)) // magic number at which accuracies and results of both approximations are equal
        return _atan_t7(d);
    else
        return _atan_p4(d);
}

// |y| must be less than 32768 (or 0.5 in fix16), x must be > 0, y must be less than y but more than 0
fix16 fast_atan2(int y, int x){
    // fix16 d = (y << 15) / x << 1; // y / x in fix16 notation
    int ya = abs(y);
    int xa = abs(x);
    fix16 r;
    if (xa > ya) {
        if (xa == 0) return 0;
        r = _atan2(ya, xa);
    } else {
        if (ya == 0) return 0;
        r = float_to_fix16(M_PI / 2) - _atan2(xa, ya);
    }
    if (x < 0)
        r = float_to_fix16(M_PI) - r;
    if (y < 0)
        r = - r;
    return r;
}

// арктангенсы оканчиваются



static inline TIM_TypeDef *tim_dev(pwm_t pwm)
{
    return pwm_config[pwm].dev;
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
/*    
    // dev->threshold_pin = UNWD_GPIO_30;                   //< Sensor "enable" pin 
    dev->silencing_pin = UNWD_GPIO_30;                   //< Silencing pin when used with 1 transceiver 
    dev->sens_pin = UNWD_GPIO_29;                //< GPIO pin on which sensor is attached 
    dev->t1_pin = UNWD_GPIO_28;              //< GPIO pin on which sensor is attached  - hi-z (GPIO_AIN) while measuring! 
    dev->t2_pin = UNWD_GPIO_27;              //< GPIO pin on which sensor is attached  -  ground while measuring! 
*/
    
    // dev->silencing_pin = UNWD_GPIO_29;                   // Silencing pin - after R10
    dev->silencing_pin = UNWD_GPIO_29;                   // disrupting pin!
    dev->t1_pin = UNWD_GPIO_24;              // not needed in current design!
    dev->t2_pin = UNWD_GPIO_30;              // ultrasound transmission via transistor - ground while measuring!
    // dev->sens_pin = UNWD_GPIO_26;                // GPIO pin on which sensor is attached
    dev->sens_pin = UNWD_GPIO_24;                // TIM2 2nd pin


    dev->transmit_pulses = UZ_TRANSMIT_PULSES;
    dev->silencing_pulses = UZ_SILENCING_PULSES;
    dev->period_us = UZ_PERIOD_US;
    // dev->period_subus = UZ_PERIOD_SUBUS;
    dev->chirp = 0;
    dev->idle_period_us = UZ_IDLE_PERIOD_US;
    dev->duty = UZ_DUTY;
    dev->duty2 = UZ_DUTY2;
    dev->verbose = false;

    // gpio_init(dev->sens_pin, GPIO_IN);
    gpio_init(dev->sens_pin, GPIO_IN_PU); // LMV331 has open collector output!
    // gpio_init(dev->threshold_pin, GPIO_IN);

    // gpio_init(dev->silencing_pin, GPIO_AIN); // for direct silencing
    gpio_init(dev->silencing_pin, GPIO_OUT); // for silencing using transistor
    gpio_init(dev->t1_pin, GPIO_AIN);
    gpio_init(dev->t2_pin, GPIO_OUT);


    // like in pwm.c
    // gpio_init(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], GPIO_OUT);
    // gpio_init_af(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], pwm_config[UMDK_PWM_0].af);
    // pwm_init
    pwm_t pwm = UMDK_PWM_0;
    // uint32_t bus_clk = periph_apb_clk(pwm_config[pwm].bus); // pwm_config is in periph_conf.h
    /* power on the used timer */
    periph_clk_en(pwm_config[pwm].bus, pwm_config[pwm].rcc_mask); // RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    /* reset configuration and CC channels */
    tim_dev(pwm)->CR1 = 0;
    tim_dev(pwm)->CR2 = 0;
    for (int i = 0; i < TIMER_CHAN; i++) {
        tim_dev(pwm)->CCR[i] = 0;
    }

    /* configure the PWM frequency and resolution by setting the auto-reload
     * and prescaler registers */
    tim_dev(pwm)->PSC = 0; // no prescaler so 32 MHz
    tim_dev(pwm)->ARR = TEST_PERIOD - 1; // should give 1 second

    /* set PWM mode */
    // tim_dev(pwm)->CCMR1 = CCMR_LEFT; // left
    // tim_dev(pwm)->CCMR1 = CCMR_RIGHT; // right - channel 1 and 2
    tim_dev(pwm)->CCMR2 = CCMR_RIGHT; // right - channel 3 and 4

    /* enable PWM outputs and start PWM generation */
#ifdef TIM_BDTR_MOE
    tim_dev(pwm)->BDTR = TIM_BDTR_MOE; // установка единицы в этот бит разрешает использовать выводы таймера как выходы.
#endif
    // tim_dev(pwm)->CCER = (TIM_CCER_CC1E); // enable one channel only!!11
    tim_dev(pwm)->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);

    // pwm_set
    /* set new value */
    // tim_dev(pwm)->CCR[UMDK_PWM_CH_3] = TEST_PERIOD / 2 + 7;
    
    

// http://chipspace.ru/stm32-general-purpose-timers-input-capture/

    // здесь ничего не меняем - как считали через UNWD_GPIO_24, так и считаем
    // впоследствии хорошо бы ещё и через UNWD_GPIO_5 счёт реализовать
    // счёт происходит посредством TIM2_CH2 - то бишь считаем тем же таймером, что и генерируем
    gpio_init(UNWD_GPIO_24, GPIO_IN); // таки надо её инициализировать!
    gpio_init_af(UNWD_GPIO_24, GPIO_AF1); // таки надо её инициализировать!
    // RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // ужо!
    TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;//Выбор активного входа. Записываем "01" в биты CC1S - связываем регистр TIM2_CCR1 со входом TI1 -
    // Note: CC2S bits are writable only when the channel is OFF (CC2E = 0 in TIMx_CCER).
    // - это PORT_A, 6 или UNWD_GPIO_27, говорят, инициализировать его не надо
    TIM2->CCMR1 &= ~(TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_3); // обнуляем на всякий пожарный
    TIM2->CCMR1 |= (TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1);//Выбор длительнотси действия фильтра - 8 тактов. IC1F = 0011.
    // TIM2->CCMR1 |= (TIM_CCMR1_IC2F_1);//Выбор длительнотси действия фильтра - 4 тактов. IC1F = 0010.
    // TIM2->CCER &= ~TIM_CCER_CC2P; // По переднему фронту - положительный перепад импульса - CCxNP и CCxP нули!
    // TIM2->CCER |= TIM_CCER_CC2P; // По заднему фронту - CCxNP 0 и CCxP 1
    TIM2->CCER |= TIM_CCER_CC2NP | TIM_CCER_CC2P; // По обоим фронтам - CCxNP 1 и CCxP 1
    TIM2->CCMR1 &= ~TIM_CCMR1_IC2PSC;//Предделитель отключен
    TIM2->CCER |= TIM_CCER_CC2E;//Разрешен захват значения счетчика в регистр TIM2_CCR1
    // TIM2->EGR |= TIM_EGR_UG; // generates update event!! otherwise first loop is performed with old prescaler!!11
    // TIM2->CR1 |= TIM_CR1_CEN;
    TIM2->SR = ~TIM_SR_UIF; // Clear the update flag for channel 1

    return 0;
}


int pulse_count = 0;
int times[UZ_MAX_PULSES] = {};

static int ultrasoundrange_transmit(ultrasoundrange_t *dev) {
    // puts ("test0");

    int T = dev->period_us; // period in sub-microseconds
    int F = 1000000 / T; // frequency in hz
    uint16_t period_ticks = dev->period_us; // ARR
    // uint16_t period_ticks = bus_clk * (dev->period_us * UZ_SUBUS_DIVISOR + dev->period_subus) / (1000000 * UZ_SUBUS_DIVISOR ); // overflow
    // uint16_t high_ticks = dev->duty; // CCR at ringing - left
    uint16_t high_ticks = period_ticks - dev->duty; // CCR at ringing - right
    // uint16_t high2_ticks = dev->duty2; // CCR at silencing - left
    uint16_t high2_ticks = period_ticks - dev->duty2; // CCR at silencing - right
    uint16_t idle_ticks = dev->idle_period_us;
    uint16_t chirp = dev->chirp;
    if (dev->verbose)
        DEBUG("[ultrasoundrange] Period : %d, frequency: %d, ticks: %d, high: %d, idle: %d, number2: %d\n", T, F, period_ticks, high_ticks, idle_ticks, dev->silencing_pulses);

    gpio_init(dev->silencing_pin, GPIO_OUT);
    gpio_clear(dev->silencing_pin); // disrupting current to op amp

    TIM2->CNT = 0;
    TIM2->ARR = period_ticks + (chirp * dev->transmit_pulses / 2) - 1;
    TIM2->CCR[UMDK_PWM_CH_3] = high_ticks;
    TIM2->SR = ~TIM_SR_UIF;
    TIM2->CR1 |= TIM_CR1_CEN;

    gpio_init(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], GPIO_OUT); // #define UNWD_GPIO_5 GPIO_PIN(PORT_A, 5)
    gpio_init_af(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], pwm_config[UMDK_PWM_0].af);
    // generation
    for (int i = 0; i < dev->transmit_pulses;){
        /*if  (TIM2->SR & TIM_SR_CC4IF) { // left
            TIM2->SR = ~TIM_SR_CC4IF;*/
        if  (TIM2->SR & TIM_SR_UIF) { // right
            TIM2->SR = ~TIM_SR_UIF;
            i++;
            TIM2->ARR -= chirp;
            // printf("q"); // успевает!
        }
    }
    // waiting
    // TIM2->ARR = period_ticks + idle_ticks - 1; // left
    TIM2->ARR = idle_ticks - 1; // right
    TIM2->CCR[UMDK_PWM_CH_3] = 65535; // right
    TIM2->SR = ~TIM_SR_UIF;
    while(!(TIM2->SR & TIM_SR_UIF));
    TIM2->SR = ~TIM_SR_UIF;
    // silencing
    TIM2->ARR = period_ticks - 1;
    if (dev->silencing_pulses < 1) {
        TIM2->CCR[UMDK_PWM_CH_3] = 0;
        gpio_set(dev->silencing_pin);
    }
    else
        TIM2->CCR[UMDK_PWM_CH_3] = high2_ticks;
    // gpio_set(dev->silencing_pin); // enabling current to op amp
    for (int i = 0; i < dev->silencing_pulses;){
        /*if  (TIM2->SR & TIM_SR_CC4IF) {
            TIM2->SR = ~TIM_SR_CC4IF;*/ // left
        if  (TIM2->SR & TIM_SR_UIF) { // right
            TIM2->SR = ~TIM_SR_UIF;
            i++;
        }
        /*if  (TIM2->SR & TIM_SR_UIF) { // left
            TIM2->SR = ~TIM_SR_UIF;
            if (i >= dev->silencing_pulses - 1) gpio_set(dev->silencing_pin); // enabling current to op amp at last pulse
        }*/
        if  (TIM2->SR & TIM_SR_CC4IF) { // right
            TIM2->SR = ~TIM_SR_CC4IF;
            if (i >= dev->silencing_pulses - 1) gpio_set(dev->silencing_pin); // enabling current to op amp at last pulse
        }
    }


/*
    TIM2->CCR[UMDK_PWM_CH_3] = 65535; // setting output to 0
    // measuring phase for adaptive silencing
    pulse_count = 0;
    int t2 = 0, total_time = 0;
    TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF);
    while ((total_time < period_ticks * 2)) {
        if (TIM2->SR & TIM_SR_CC2IF) {
            t2 = TIM2->CCR[1];
            TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (period_ticks / 2))) t2 += period_ticks;
            times[pulse_count] = t2 + total_time;
            pulse_count++;
        }
        if (TIM2->SR & TIM_SR_CC2OF) {
            t2 = TIM2->CCR[1];
            // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (period_ticks / 2))) t2 += period_ticks;
            times[pulse_count] = t2 + total_time;
            pulse_count++;
            printf("q");
        }
        if (TIM2->SR & TIM_SR_UIF) { // when the counter reaches ARR
            if (TIM2->SR & TIM_SR_CC2IF) {
                t2 = TIM2->CCR[1];
                if (t2 > (period_ticks / 2)) {
                    TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
                    times[pulse_count] = t2 + total_time;
                    pulse_count++;
                }
            }
            if (TIM2->SR & TIM_SR_CC2OF) {
                t2 = TIM2->CCR[1];
                if (t2 > (period_ticks / 2)) {
                    // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    times[pulse_count] = t2 + total_time;
                    pulse_count++;
                }
            }
            total_time += period_ticks;
            TIM2->SR = ~TIM_SR_UIF; // Clear the update flag for channel 1
        }
    }


    // phase calculation
    int cos_prod = 0, sin_prod = float_to_fix16(1.), sign = 1; // TODO: determine sign by reading pin value!
    int coef = ((unsigned int)(1 << 31) / period_ticks) << 1;
    for (int i = 0; i < pulse_count; i++){ // integral of product of signal and cosine
        // cos_prod += 2 * fast_sin(times[i] / period_ticks * (1 << 32));
        cos_prod += 2 * sign * fast_sin(times[i] * coef);
        sign = -sign;
    }
    sign = 1; // TODO: determine sign by reading pin value!
    for (int i = 0; i < pulse_count; i++){ // integral of product of signal and sine
        // sin_prod -= 2 * sign * fast_cos(times[i] / period_ticks * (1 << 32));
        sin_prod -= 2 * sign * fast_cos(times[i] * coef);
        sign = -sign;
    }
    sin_prod -= sign * float_to_fix16(1.); // cos we integrate over full period, cos is 1 at its ends, so we should add it with proper sign. Sin(0) is 0 so we may not add it



    printf("[umdk-opt3001-uz] Pulse number : %d\n[umdk-opt3001-uz] Pulse times:", pulse_count);
    for (int i = 0; i < pulse_count; i++){
        printf(" %d", times[i]);
    }
    printf("\n");
    printf("Cos: %d, sin: %d\n", cos_prod, sin_prod);
*/


    // TIM2->CCR[UMDK_PWM_CH_3] = 0; // end of generation, but not the end of the timer - left
    TIM2->CCR[UMDK_PWM_CH_3] = 65535; // end of generation, but not the end of the timer - right
    TIM2->ARR = 65535; // end of generation, but not the end of the timer
    // gpio_init(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], GPIO_AIN); // end of generation, but not the end of the timer
    gpio_init_af(pwm_config[UMDK_PWM_0].pins[UMDK_PWM_CH_3], 0); // end of generation, but not the end of the timer
    // TIM2->CR1 &= ~TIM_CR1_CEN;
    gpio_init(dev->silencing_pin, GPIO_AIN); // for filtering noise from processor
    // end

    // switch_time += T;
    // while((xtimer_now_usec() - begin_time) < switch_time); // waiting for gate to charge


    int begin_time = xtimer_now_usec(); // we are always subtracting this time to prevent integer overflow
    // ready to measure?
    return begin_time;
}

#if 0
static uint32_t ultrasoundrange_count_pulses(ultrasoundrange_t *dev, int begin_time) {
    // pwm_t pwm = UMDK_PWM_0;
    // uint32_t bus_clk = periph_apb_clk(pwm_config[pwm].bus); // pwm_config is in periph_conf.h


    // xtimer_spin(xtimer_ticks_from_usec(4000)); // waiting for transmitter to relax
 
    pulse_count = 0;
    // int pulse_count = 0;
    // static int times[UZ_MAX_PULSES] = {};
    int timeout_us = 0;

    // TIM2->ARR = 65535; // отчего-то не считает в первый период!!1
    int arr_period = TIM2->ARR + 1;

    int t1 = 0, t2 = 0, total_time = 0;
    TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF);
    while ((pulse_count < UZ_MAX_PULSES) && (timeout_us < 1000 * UZ_MAX_TIMEOUT_MS)) {
        if (TIM2->SR & TIM_SR_CC2IF) {
            t2 = TIM2->CCR[1];
            TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (arr_period / 2))) t2 += arr_period;
            t2 += total_time;
            times[pulse_count] = t2 - t1;
            pulse_count++;
            t1 = t2;
        }
        if (TIM2->SR & TIM_SR_CC2OF) {
            t2 = TIM2->CCR[1];
            // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (arr_period / 2))) t2 += arr_period;
            t2 += total_time;
            times[pulse_count] = t2 - t1;
            pulse_count++;
            t1 = t2;
            // printf("q");
        }
        if (TIM2->SR & TIM_SR_UIF) { // when the counter reaches ARR
            if (TIM2->SR & TIM_SR_CC2IF) {
                t2 = TIM2->CCR[1];
                if (t2 > (arr_period / 2)) {
                    TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
                    t2 += total_time;
                    times[pulse_count] = t2 - t1;
                    t1 = t2;
                    pulse_count++;
                }
            }
            if (TIM2->SR & TIM_SR_CC2OF) {
                t2 = TIM2->CCR[1];
                if (t2 > (arr_period / 2)) {
                    // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    t2 += total_time;
                    times[pulse_count] = t2 - t1;
                    t1 = t2;
                    pulse_count++;
                }
            }
            total_time += arr_period;
            TIM2->SR = ~TIM_SR_UIF; // Clear the update flag for channel 1
        }
        timeout_us++;
    }
    TIM2->CR1 &= ~TIM_CR1_CEN; // turning timer off

    // if (dev->verbose && (pulse_count > UZ_MAX_PULSES)) { // never executed
    if (dev->verbose) {
        /*
        printf("[umdk-opt3001-uz] Pulse number : %d\n[umdk-opt3001-uz] Pulse times:", pulse_count);
        for (int i = 0; i < pulse_count; i++){
            if (times[i] != -1) printf(" %d", times[i]);
        }
        printf("\n");
        */
        DEBUG("[ultrasoundrange] Pulse number : %d\n[umdk-opt3001-uz] Pulse times:", pulse_count);
        for (int i = 0; i < pulse_count; i++){
            DEBUG(" %d", times[i]);
        }
        DEBUG("\n");
    }
    // printf("clock: %d\n", (int)periph_apb_clk(pwm_config[UMDK_PWM_0].bus)); // 32000000
    return pulse_count;
}
#endif


static int32_t ultrasoundrange_first_echo(ultrasoundrange_t *dev, int listening_time) {

    int max_pulses = 10;
    int min_time = 32000 * 3; // 3 ms
    int max_ticks = dev->period_us * max_pulses * 3 / 4;
    for (int i = 0; i < max_pulses; i++)
        times[i] = -max_ticks * 2; 
    
    pulse_count = 0;
    // int pulse_count = 0;
    // static int times[UZ_MAX_PULSES] = {};
    int timeout_us = 0;

    // TIM2->ARR = 65535; // отчего-то не считает в первый период!!1
    int arr_period = TIM2->ARR + 1;

    int t2 = 0, total_time = 0;
    TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF);
    while ((pulse_count < UZ_MAX_PULSES) && (timeout_us < 1000 * UZ_MAX_TIMEOUT_MS)) {
        if (TIM2->SR & TIM_SR_CC2IF) {
            t2 = TIM2->CCR[1];
            TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (arr_period / 2))) t2 += arr_period;
            t2 += total_time;
            // pulse_count++;
            if ((times[pulse_count] > t2 - max_ticks) && (times[pulse_count] > min_time))
                return times[pulse_count];
            times[pulse_count] = t2;
            pulse_count = (pulse_count + 1) % max_pulses;
        }
        if (TIM2->SR & TIM_SR_CC2OF) {
            t2 = TIM2->CCR[1];
            // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
            if ((TIM2->SR & TIM_SR_UIF) && (t2 < (arr_period / 2))) t2 += arr_period;
            t2 += total_time;
            if ((times[pulse_count] > t2 - max_ticks) && (times[pulse_count] > min_time))
                return times[pulse_count];
            times[pulse_count] = t2;
            pulse_count = (pulse_count + 1) % max_pulses;
        }
        if (TIM2->SR & TIM_SR_UIF) { // when the counter reaches ARR
            if (TIM2->SR & TIM_SR_CC2IF) {
                t2 = TIM2->CCR[1];
                if (t2 > (arr_period / 2)) {
                    TIM2->SR = ~TIM_SR_CC2IF; // значение скопировали - флаг можно и снять
                    t2 += total_time;
                    if ((times[pulse_count] > t2 - max_ticks) && (times[pulse_count] > min_time))
                        return times[pulse_count];
                    times[pulse_count] = t2;
                    pulse_count = (pulse_count + 1) % max_pulses;
                }
            }
            if (TIM2->SR & TIM_SR_CC2OF) {
                t2 = TIM2->CCR[1];
                if (t2 > (arr_period / 2)) {
                    // TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    TIM2->SR = ~ (TIM_SR_CC2OF); // значение скопировали - флаг можно и снять
                    t2 += total_time;
                    if ((times[pulse_count] > t2 - max_ticks) && (times[pulse_count] > min_time))
                        return times[pulse_count];
                    times[pulse_count] = t2;
                    pulse_count = (pulse_count + 1) % max_pulses;
                }
            }
            total_time += arr_period;
            TIM2->SR = ~TIM_SR_UIF; // Clear the update flag for channel 1
        }
        timeout_us++;
    }
    TIM2->CR1 &= ~TIM_CR1_CEN; // turning timer off

    return -1;
}

#if 0
// count pulses without recording their times
static uint32_t ultrasoundrange_simple_count_pulses(ultrasoundrange_t *dev, int listening_time) {

    pulse_count = 0;
    // TIM2->ARR = 65535; // отчего-то не считает в первый период!!1
    int arr_period = TIM2->ARR + 1;
    int total_time = 0;

    TIM2->SR = ~ (TIM_SR_CC2IF | TIM_SR_CC2OF);
    while (total_time < listening_time) {
        if (TIM2->SR & TIM_SR_CC2IF) {
            TIM2->SR = ~TIM_SR_CC2IF;
            pulse_count++;
        }
        if (TIM2->SR & TIM_SR_CC2OF) {
            TIM2->SR = ~ (TIM_SR_CC2OF);
            pulse_count++;
        }
        if (TIM2->SR & TIM_SR_UIF) { // when the counter reaches ARR
            total_time += arr_period;
            TIM2->SR = ~TIM_SR_UIF; // Clear the update flag for channel 1
        }
    }
    TIM2->CR1 &= ~TIM_CR1_CEN; // turning timer off

    printf("[usoundrange] Pulse number : %d in %d ticks\n", pulse_count, listening_time);
    return -1;
}
#endif



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
    measure->range = 42;
    
    assert(dev != NULL);

    // count_pulses(dev, transmit(dev));
    // transmit(dev); simple_count_pulses(dev, 320000);
    ultrasoundrange_transmit(dev); int t = ultrasoundrange_first_echo(dev, 640000); // distance_in_mm = time_in_ticks * 1000 ms / 32000000 Hz / 2 * 330 m/s
    if (t > 0)
        measure->range = t / 194;
    else
        measure->range = -1;

    return 0;
}


#ifdef __cplusplus
}
#endif
