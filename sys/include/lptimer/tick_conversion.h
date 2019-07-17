/*
 * Copyright (C) 2016 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup sys_lptimer
 *
 * @{
 * @file
 * @brief   lptimer tick <-> seconds conversions for different values of LPTIMER_HZ
 * @author  Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

#ifndef LPTIMER_TICK_CONVERSION_H
#define LPTIMER_TICK_CONVERSION_H

#ifndef LPTIMER_H
#error "Do not include this file directly! Use lptimer.h instead"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if defined (LPTIMER_HZ)
static inline uint32_t _lptimer_ticks_from_msec(uint32_t msec) {
    return (msec*LPTIMER_HZ)/1000;
}

static inline uint64_t _lptimer_ticks_from_msec64(uint64_t msec) {
    return (msec*LPTIMER_HZ)/1000;
}

static inline uint32_t _lptimer_msec_from_ticks(uint32_t ticks) {
    return ((uint64_t)ticks * 1000)/LPTIMER_HZ;
}

static inline uint64_t _lptimer_msec_from_ticks64(uint64_t ticks) {
    return ((uint64_t)ticks * 1000)/LPTIMER_HZ;
}
#else
#error Unknown hardware timer frequency (LPTIMER_HZ), check board.h and/or add an implementation in sys/include/lptimer/tick_conversion.h
#endif

#ifdef __cplusplus
}
#endif

#endif /* LPTIMER_TICK_CONVERSION_H */
