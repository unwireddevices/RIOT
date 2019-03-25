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

#include <math.h>

#include "dsp/int_fft_q15.h"

#include "dsp/twiddle_coef.h"
#include "dsp/int_sqrt.h"
#include "dsp/int_mult_q15.h"


uint32_t log_base_2 (uint32_t n)
{
    uint16_t logValue = -1;

    if (n == 0) {
        return logValue;
    }

    logValue = 0;
    while (n >>= 1) {
        ++logValue;
    }

    return logValue;
}


int32_t fft_q15(q15_t fr[], q15_t fi[], int32_t m, int32_t inverse)
{
    int32_t mr, nn, i, j, l, k, istep, n, scale, shift;
    int16_t qr, qi, tr, ti, wr, wi;

    n = 1 << m;

    /* max FFT size = N_WAVE */
    if (n > N_WAVE) {
        return -1;
    }

    mr = 0;
    nn = n - 1;
    scale = 0;

    /* decimation in time - re-order data */
    for (m = 1; m <= nn; ++m) {
        l = n;
        do {
            l >>= 1;
        } while (mr + l > nn);
        mr = (mr & (l - 1)) + l;

        if (mr <= m) {
            continue;
        }
        tr = fr[m];
        fr[m] = fr[mr];
        fr[mr] = tr;
        ti = fi[m];
        fi[m] = fi[mr];
        fi[mr] = ti;
    }

    l = 1;
    k = LOG2_N_WAVE - 1;
    while (l < n) {
        if (inverse) {
            /* variable scaling, depending upon data */
            shift = 0;
            for (i = 0; i < n; ++i) {
                j = fr[i];
                if (j < 0) {
                    j = -j;
                }
                m = fi[i];
                if (m < 0) {
                    m = -m;
                }
                if (j > 16383 || m > 16383) {
                    shift = 1;
                    break;
                }
            }
            if (shift) {
                ++scale;
            }
        } else {
            /*
              fixed scaling, for proper normalization --
              there will be log2(n) passes, so this results
              in an overall factor of 1/n, distributed to
              maximize arithmetic accuracy.
            */
            shift = 1;
        }
        /*
          it may not be obvious, but the shift will be
          performed on each data point exactly once,
          during this pass.
        */
        istep = l << 1;
        for (m = 0; m < l; ++m) {
            j = m << k;
            /* 0 <= j < N_WAVE/2 */
            wr =  Sinewave[j + N_WAVE / 4];     //cos
            wi = -Sinewave[j];                  //sin
            if (inverse) {
                wi = -wi;
            }
            if (shift) {
                wr >>= 1;
                wi >>= 1;
            }
            for (i = m; i < n; i += istep) {
                j = i + l;
                tr = int_mult_q15(wr, fr[j]) - int_mult_q15(wi, fi[j]);
                ti = int_mult_q15(wr, fi[j]) + int_mult_q15(wi, fr[j]);
                qr = fr[i];
                qi = fi[i];
                if (shift) {
                    qr >>= 1;
                    qi >>= 1;
                }
                fr[j] = qr - tr;
                fi[j] = qi - ti;
                fr[i] = qr + tr;
                fi[i] = qi + ti;
            }
        }
        --k;
        l = istep;
    }
    return scale;
}


void fft_magnitude_q15 (const q15_t *fx, uint32_t fft_size, uint16_t *mag)
{
    for (uint32_t i = 0; i < fft_size; i++) {
        mag[i] = int_sqrt_16((uint32_t)fx[i] * (uint32_t)fx[i] + (uint32_t)fx[fft_size + i] * (uint32_t)fx[fft_size + i]);
    }

    for (uint32_t i = 0; i < fft_size / 2; i++) {
        if (i != 0) {
            mag[i] <<= 1; //restore amplitude
        }
    }
}

void hanning_window_q15(q15_t fr[], int32_t n)
{
    int32_t i, j, k;
    int16_t xx[N_WAVE];

    j = N_WAVE / n;
    n >>= 1;

    for (i = 0, k = N_WAVE / 4; i < n; ++i, k += j) {
        xx[i] = 16384 - (Sinewave[k] >> 1);

    }
    n <<= 1;
    for (k -= j; i < n; ++i, k -= j) {
        xx[i] = 16384 - (Sinewave[k] >> 1);
    }

    for (i = 0; i < n; ++i) {
        fr[i] = int_mult_q15(fr[i], xx[i]);
    }
}

uint32_t fft_bin_max_q15(const uint16_t *data, uint32_t length, uint16_t *max_value)
{
    uint32_t cnt = 0;
    uint32_t pos = 0;

    *max_value = data[pos];
    for (cnt = 0; cnt < length; cnt++) {
        if (*max_value < data[cnt]) {
            *max_value = data[cnt];
            pos = cnt;
        }
    }
    return pos;
}

float32_t fft_bin_to_freq(uint32_t num_fft_bin, uint32_t freq_sample_rate, uint32_t fft_size)
{
    return (float32_t)(num_fft_bin * (float32_t)(freq_sample_rate / 2) / (float32_t)(fft_size / 2));
}

uint32_t freq_to_fft_bin(uint32_t freq, uint32_t freq_sample_rate, uint32_t fft_size)
{
    return (uint32_t)round((float32_t)(fft_size * (float32_t)freq / (float32_t)freq_sample_rate));
}