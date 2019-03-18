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
#ifndef FFT_Q15
#define FFT_Q15

#include "dsp/dsp_type.h"



/**
 * @brief           Perform forward/inverse FFT
 * 
 * @param[in/out]   fr - real data arrays  0 <= n < 2**m
 * @param[in/out]   fi - imaginary data arrays  0 <= n < 2**m
 * @param[in]       m  - size data
 * @param[in]       inverse  - 0 for forward transform (FFT), or 1 for iFFT
 * @return          scale for inverse FFT
 */
int32_t fft_q15(q15_t fr[], q15_t fi[], int32_t m, int32_t inverse);

/**
 * @brief [brief description]
 * 
 * @param fr [description]
 * @param n [description]
 */
void hanning_window_q15(q15_t fr[], int32_t n);

/**
 * @brief [brief description]
 * 
 * @param fx [description]
 * @param fft_size [description]
 * @param mag [description]
 * @return [description]
 */
void fft_magnitude_q15 (const q15_t *fx, uint32_t fft_size, uint16_t *mag);

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param n [description]
 * @return [description]
 */
uint32_t log_base_2 (uint32_t n);

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param data [description]
 * @param length [description]
 * @param max_value [description]
 * @return [description]
 */
uint32_t fft_bin_max_q15(uint16_t *data, uint32_t length, uint16_t *max_value);

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param num_fft_bin [description]
 * @param freq_sample_rate [description]
 * @param fft_size [description]
 * @return [description]
 */
float32_t fft_bin_to_freq(uint32_t num_fft_bin, uint32_t freq_sample_rate, uint32_t fft_size);

/**
 * @brief [brief description]
 * @details [long description]
 * 
 * @param freq [description]
 * @param freq_sample_rate [description]
 * @param fft_size [description]
 * @return [description]
 */
uint32_t freq_to_fft_bin(uint32_t freq, uint32_t freq_sample_rate, uint32_t fft_size);

#endif