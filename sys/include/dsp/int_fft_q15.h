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
 * 
 * @return          scale for inverse FFT
 */
int32_t fft_q15(q15_t fr[], q15_t fi[], int32_t m, int32_t inverse);

/**
 * @brief Apply Hanning Window of raw data 
 * 
 * @param fr - real data arrays  0 <= n < 2**m
 * @param m  - size data
 */
void hanning_window_q15(q15_t fr[], int32_t n);

/**
 * @brief The calculation of the amplitude according to the FFT
 * 
 * @param[in] fx - FFT dataset
 * @param[in] fft_size - size data
 * @param[out] mag - amplitude array for frequency bin
 */
void fft_magnitude_q15 (const q15_t *fx, uint32_t fft_size, uint16_t *mag);

/**
 * @brief Calculation of the logarithm of base 2
 * 
 * @param[in] n fixed point value
 * 
 * @return logarithm value
 */
uint32_t log_base_2 (uint32_t n);

/**
 * @brief Search for the maximum magnitude value in the FFT data
 * 
 * @param[in]  data - FFT dataset
 * @param[in]  length - size data
 * @param[out] max_value -maximum magnitude
 * 
 * @return frequency bin for maximum magnitude
 */
uint32_t fft_bin_max_q15(const uint16_t *data, uint32_t length, uint16_t *max_value);

/**
 * @brief Conversion from FFT frequency bin to frequency
 * 
 * @param num_fft_bin [description]
 * @param freq_sample_rate [description]
 * @param fft_size [description]
 * 
 * @return [description]
 */
float32_t fft_bin_to_freq(uint32_t num_fft_bin, uint32_t freq_sample_rate, uint32_t fft_size);

/**
 * @brief Conversion from frequency to FFT frequency bin
 * @details [long description]
 * 
 * @param freq [description]
 * @param freq_sample_rate [description]
 * @param fft_size [description]
 * 
 * @return [description]
 */
uint32_t freq_to_fft_bin(uint32_t freq, uint32_t freq_sample_rate, uint32_t fft_size);

#endif