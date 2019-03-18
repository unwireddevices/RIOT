/*
 * Copyright (C) 2016-2019 Unwired Devices
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     sys_dsp
 * @{
 *
 * @file
 * @brief       Fixed-point in-place Fast Fourier Transform
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 *
 * @}
 */
#ifndef INT_SQRT_H
#define INT_SQRT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <limits.h>


// #define BITS_PER_LONG 32

// static uint32_t __fls(uint32_t word) {
//     int num = BITS_PER_LONG - 1;

// #if BITS_PER_LONG == 64
//     if (!(word & (~0ul << 32))) {
//         num -= 32;
//         word <<= 32;
//     }
// #endif
//     if (!(word & (~0ul << (BITS_PER_LONG - 16)))) {
//         num -= 16;
//         word <<= 16;
//     }
//     if (!(word & (~0ul << (BITS_PER_LONG - 8)))) {
//         num -= 8;
//         word <<= 8;
//     }
//     if (!(word & (~0ul << (BITS_PER_LONG - 4)))) {
//         num -= 4;
//         word <<= 4;
//     }
//     if (!(word & (~0ul << (BITS_PER_LONG - 2)))) {
//         num -= 2;
//         word <<= 2;
//     }
//     if (!(word & (~0ul << (BITS_PER_LONG - 1))))
//         num -= 1;
//     return num;
// }

// static int fls(int x) {
//     int r = 32;

//     if (!x)
//         return 0;
//     if (!(x & 0xffff0000u)) {
//         x <<= 16;
//         r -= 16;
//     }
//     if (!(x & 0xff000000u)) {
//         x <<= 8;
//         r -= 8;
//     }
//     if (!(x & 0xf0000000u)) {
//         x <<= 4;
//         r -= 4;
//     }
//     if (!(x & 0xc0000000u)) {
//         x <<= 2;
//         r -= 2;
//     }
//     if (!(x & 0x80000000u)) {
//         x <<= 1;
//         r -= 1;
//     }
//     return r;
// }

// #if BITS_PER_LONG == 32

// static int fls64(uint64_t x) {
//     uint32_t h = x >> 32;
//     if (h)
//         return fls(h) + 32;
//     return fls(x);
// }
// #elif BITS_PER_LONG == 64

// static int fls64(uint64_t x) {
//     if (x == 0)
//         return 0;
//     return __fls(x) + 1;
// }
// #else
// #error BITS_PER_LONG not 32 or 64
// #endif

// /**
//  * int_sqrt - computes the integer square root
//  * @x: integer of which to calculate the sqrt
//  *
//  * Computes: floor(sqrt(x))
//  */
// uint32_t int_sqrt(uint32_t x) {
//     uint32_t b;
//     uint32_t m;
//     uint32_t y = 0;

//     if (x <= 1)
//         return x;



//     m = 1UL << (__fls(x) & ~1UL);
//     while (m != 0) {
//         b = y + m;
//         y >>= 1;

//         if (x >= b) {
//             x -= b;
//             y += m;
//         }
//         m >>= 2;
//     }

//     return y;
// }

// #if BITS_PER_LONG < 64

// /**
//  * int_sqrt64 - strongly typed int_sqrt function when minimum 64 bit input
//  * is expected.
//  * @x: 64bit integer of which to calculate the sqrt
//  */
// uint32_t int_sqrt64(uint64_t x) {
//     uint64_t b;
//     uint64_t m;
//     uint64_t y = 0;

//     if (x <= ULONG_MAX)
//         return int_sqrt((uint32_t) x);

//     m = 1ULL << ((fls64(x) - 1) & ~1ULL);
//     while (m != 0) {
//         b = y + m;
//         y >>= 1;

//         if (x >= b) {
//             x -= b;
//             y += m;
//         }
//         m >>= 2;
//     }

//     return y;
// }
// #endif

uint32_t int_sqrt_32(uint32_t n);


uint16_t int_sqrt_16(uint16_t n);


uint8_t int_sqrt_8(uint8_t n);


#ifdef __cplusplus
}
#endif

#endif /* INT_SQRT_H */

