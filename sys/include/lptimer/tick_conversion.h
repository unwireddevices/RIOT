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

#include "div.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Some optimizations for common timer frequencies */
#if (LPTIMER_SHIFT != 0)
#if (LPTIMER_HZ % 15625 != 0)
#error LPTIMER_HZ must be a multiple of 15625 (5^6) when using LPTIMER_SHIFT
#endif
#if (LPTIMER_HZ > 1000000ul)
#if (LPTIMER_HZ != (1000000ul << LPTIMER_SHIFT))
#error LPTIMER_HZ != (1000000ul << LPTIMER_SHIFT)
#endif
/* LPTIMER_HZ is a power-of-two multiple of 1 MHz */
/* e.g. cc2538 uses a 16 MHz timer */
static inline uint32_t _lptimer_ticks_from_usec(uint32_t usec) {
    return (usec << LPTIMER_SHIFT); /* multiply by power of two */
}

static inline uint64_t _lptimer_ticks_from_usec64(uint64_t usec) {
    return (usec << LPTIMER_SHIFT); /* multiply by power of two */
}

static inline uint32_t _lptimer_usec_from_ticks(uint32_t ticks) {
    return (ticks >> LPTIMER_SHIFT); /* divide by power of two */
}

static inline uint64_t _lptimer_usec_from_ticks64(uint64_t ticks) {
    return (ticks >> LPTIMER_SHIFT); /* divide by power of two */
}

#else /* !(LPTIMER_HZ > 1000000ul) */
#if ((LPTIMER_HZ << LPTIMER_SHIFT) != 1000000ul)
#error (LPTIMER_HZ << LPTIMER_SHIFT) != 1000000ul
#endif
/* 1 MHz is a power-of-two multiple of LPTIMER_HZ */
/* e.g. ATmega2560 uses a 250 kHz timer */
static inline uint32_t _lptimer_ticks_from_usec(uint32_t usec) {
    return (usec >> LPTIMER_SHIFT); /* divide by power of two */
}

static inline uint64_t _lptimer_ticks_from_usec64(uint64_t usec) {
    return (usec >> LPTIMER_SHIFT); /* divide by power of two */
}

static inline uint32_t _lptimer_usec_from_ticks(uint32_t ticks) {
    return (ticks << LPTIMER_SHIFT); /* multiply by power of two */
}

static inline uint64_t _lptimer_usec_from_ticks64(uint64_t ticks) {
    return (ticks << LPTIMER_SHIFT); /* multiply by power of two */
}
#endif /* defined(LPTIMER_SHIFT) && (LPTIMER_SHIFT != 0) */
#elif LPTIMER_HZ == (1000000ul)
/* This is the most straightforward as the lptimer API is based around
 * microseconds for representing time values. */
static inline uint32_t _lptimer_usec_from_ticks(uint32_t ticks) {
    return ticks; /* no-op */
}

static inline uint64_t _lptimer_usec_from_ticks64(uint64_t ticks) {
    return ticks; /* no-op */
}

static inline uint32_t _lptimer_ticks_from_usec(uint32_t usec) {
    return usec; /* no-op */
}

static inline uint64_t _lptimer_ticks_from_usec64(uint64_t usec) {
    return usec; /* no-op */
}

#elif LPTIMER_HZ == (32768ul)
/* This is a common frequency for RTC crystals. We use the fact that the
 * greatest common divisor between 32768 and 1000000 is 64, so instead of
 * multiplying by the fraction (32768 / 1000000), we will instead use
 * (512 / 15625), which reduces the truncation caused by the integer widths */
static inline uint32_t _lptimer_ticks_from_usec(uint32_t usec) {
    return div_u32_by_15625div512(usec);
}

static inline uint64_t _lptimer_ticks_from_usec64(uint64_t usec) {
    return div_u64_by_15625div512(usec);
}

static inline uint32_t _lptimer_usec_from_ticks(uint32_t ticks) {
    /* return (usec * 15625) / 512; */
    /* Using 64 bit multiplication to avoid truncating the top 9 bits */
    uint64_t usec = (uint64_t)ticks * 15625ul;
    return (usec >> 9); /* equivalent to (usec / 512) */
}

static inline uint64_t _lptimer_usec_from_ticks64(uint64_t ticks) {
    /* return (usec * 15625) / 512; */
    uint64_t usec = (uint64_t)ticks * 15625ul;
    return (usec >> 9); /* equivalent to (usec / 512) */
}

#else
/* No matching implementation found, try to give meaningful error messages */
#if ((LPTIMER_HZ % 15625) == 0)
#error Unsupported hardware timer frequency (LPTIMER_HZ), missing LPTIMER_SHIFT in board.h? See lptimer.h documentation for more info
#else
#error Unknown hardware timer frequency (LPTIMER_HZ), check board.h and/or add an implementation in sys/include/lptimer/tick_conversion.h
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* LPTIMER_TICK_CONVERSION_H */
