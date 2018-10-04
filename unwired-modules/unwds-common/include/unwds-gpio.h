/*
 * Copyright (C) 2016-2018 Unwired Devices LLC <info@unwds.com>

 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
 
 /**
 * @defgroup    
 * @ingroup     
 * @brief       
 * @{
 * @file		unwds-gpio.h
 * @brief       GPIO map
 * @author      Eugene Ponomarev
 */

#ifndef UNWDS_GPIO_H_
#define UNWDS_GPIO_H_

static const gpio_t unwds_gpio_map[] = {
#ifdef UNWD_GPIO_0
    UNWD_GPIO_0,
#else
    0,
#endif
#ifdef UNWD_GPIO_1
    UNWD_GPIO_1,
#else
    0,
#endif
#ifdef UNWD_GPIO_2
    UNWD_GPIO_2,
#else
    0,
#endif
#ifdef UNWD_GPIO_3
    UNWD_GPIO_3,
#else
    0,
#endif
#ifdef UNWD_GPIO_4
    UNWD_GPIO_4,
#else
    0,
#endif
#ifdef UNWD_GPIO_5
    UNWD_GPIO_5,
#else
    0,
#endif
#ifdef UNWD_GPIO_6
    UNWD_GPIO_6,
#else
    0,
#endif
#ifdef UNWD_GPIO_7
    UNWD_GPIO_7,
#else
    0,
#endif
#ifdef UNWD_GPIO_8
    UNWD_GPIO_8,
#else
    0,
#endif
#ifdef UNWD_GPIO_9
    UNWD_GPIO_9,
#else
    0,
#endif
#ifdef UNWD_GPIO_10
    UNWD_GPIO_10,
#else
    0,
#endif
#ifdef UNWD_GPIO_11
    UNWD_GPIO_11,
#else
    0,
#endif
#ifdef UNWD_GPIO_12
    UNWD_GPIO_12,
#else
    0,
#endif
#ifdef UNWD_GPIO_13
    UNWD_GPIO_13,
#else
    0,
#endif
#ifdef UNWD_GPIO_14
    UNWD_GPIO_14,
#else
    0,
#endif
#ifdef UNWD_GPIO_15
    UNWD_GPIO_15,
#else
    0,
#endif
#ifdef UNWD_GPIO_16
    UNWD_GPIO_16,
#else
    0,
#endif
#ifdef UNWD_GPIO_17
    UNWD_GPIO_17,
#else
    0,
#endif
#ifdef UNWD_GPIO_18
    UNWD_GPIO_18,
#else
    0,
#endif
#ifdef UNWD_GPIO_19
    UNWD_GPIO_19,
#else
    0,
#endif
#ifdef UNWD_GPIO_20
    UNWD_GPIO_20,
#else
    0,
#endif
#ifdef UNWD_GPIO_21
    UNWD_GPIO_21,
#else
    0,
#endif
#ifdef UNWD_GPIO_22
    UNWD_GPIO_22,
#else
    0,
#endif
#ifdef UNWD_GPIO_23
    UNWD_GPIO_23,
#else
    0,
#endif
#ifdef UNWD_GPIO_24
    UNWD_GPIO_24,
#else
    0,
#endif
#ifdef UNWD_GPIO_25
    UNWD_GPIO_25,
#else
    0,
#endif
#ifdef UNWD_GPIO_26
    UNWD_GPIO_26,
#else
    0,
#endif
#ifdef UNWD_GPIO_27
    UNWD_GPIO_27,
#else
    0,
#endif
#ifdef UNWD_GPIO_28
    UNWD_GPIO_28,
#else
    0,
#endif
#ifdef UNWD_GPIO_29
    UNWD_GPIO_29,
#else
    0,
#endif
#ifdef UNWD_GPIO_30
    UNWD_GPIO_30,
#else
    0,
#endif
#ifdef UNWD_GPIO_31
    UNWD_GPIO_31,
#else
    0,
#endif
};

#endif