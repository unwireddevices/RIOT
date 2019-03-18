/*
  Short test program to accompany fix_fft.c
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "ps.h"

#include "dsp/int_fft_q15.h"

#ifndef M_PI
#   define M_PI 3.14159265359
#endif

#if 1
int main(void) {

    ps();
    
    uint32_t i;
    const uint32_t freq_sample = 400;
    const uint32_t fsine = 50;
    int16_t fx[FFT_SAMPLE * 2];
    
    

    for (i = FFT_SAMPLE; i < FFT_SAMPLE * 2; i++) {
        fx[i] = 0;
    }

    for (i = 0; i < FFT_SAMPLE; i++) {
        fx[i] = (32767 * sin(i * 2 * M_PI * fsine / freq_sample));
        fx[i] += (32767 / 3 * sin(i * 2 * M_PI * fsine * 3 / freq_sample));
        fx[i] += (32767 / 5 * sin(i * 2 * M_PI * fsine * 5 / freq_sample));
    }

    hanning_window_q15(fx, FFT_SAMPLE);
    
    int32_t scale;
    scale = fft_q15(fx, fx + FFT_SAMPLE, log_base_2(FFT_SAMPLE), 0);
    printf("%ld\n", scale);

    uint16_t mag[FFT_SAMPLE];
    fft_magnitude_q15(fx, FFT_SAMPLE, mag);
    
 
    uint32_t fft_bin  = freq_to_fft_bin(freq_sample/2, freq_sample, FFT_SAMPLE);
    printf("fft bin for %ldHz is %ld\n", freq_sample/2, fft_bin);
    
    for(i = 0; i < fft_bin + 1 ; ++i) {
        printf("[%ld]\t%f[Hz]\t%hu\n", i, (float)(i * (float)(freq_sample / 2) / (float)(FFT_SAMPLE / 2)), mag[i]);
    }
       
    printf("Maximum magnitude: ");
    uint16_t max_mag;
    uint32_t fft_bin_max =  fft_bin_max_q15(mag, FFT_SAMPLE, &max_mag);
    printf("%ld[Hz]\t%hu\n", (uint32_t)fft_bin_to_freq(fft_bin_max, freq_sample, FFT_SAMPLE), max_mag);
    printf("HELLO\n");
    ps();
    return 0;
}
#else
#include "signal.h"

int main(void) {
    const int freq_sample = FREQ_SAMPLE;
    short fx[FFT_SAMPLE * 2];
    unsigned short mag[FFT_SAMPLE];
    int scale;

    for (uint32_t i = FFT_SAMPLE; i < FFT_SAMPLE * 2; i++) {
        fx[i] = 0;
    }

    for (uint32_t i = 0; i < FFT_SAMPLE; i++) {
        fx[i] = signal[i];
    }

    //hanning_window_q15(fx, FFT_SAMPLE);
    
    printf("Input signal:\n");
    for (int i = 0; i < FFT_SAMPLE; i++) {
        printf("%d\t\t%hd\n", i, fx[i]);
    }
    printf("fft:\n");
    scale = fft_q15(fx, fx + FFT_SAMPLE, log_base_2(FFT_SAMPLE), 0);
    printf("scale = %d\n", scale);

    fft_magnitude_q15(fx, FFT_SAMPLE, mag);
    
    uint32_t fft_bin  = freq_to_fft_bin(freq_sample/2, freq_sample, FFT_SAMPLE);
    printf("fft bin for %dHz is %ld\n", freq_sample/2, fft_bin);
    
    for(uint32_t i = 0; i < fft_bin + 1; ++i) {
        printf("[%ld]\t%f[Hz]\t%hu\n", i, (float) (i * (float) (freq_sample / 2) / (float) (FFT_SAMPLE / 2)), mag[i]);
    }
    
    
    printf("Maximum magnitude: ");
    uint16_t max_mag;
    uint32_t fft_bin_max =  fft_bin_max_q15(mag, FFT_SAMPLE, &max_mag);
    printf("[%ld]\t%f[Hz]\t%hu\n", fft_bin_max, fft_bin_to_freq(fft_bin_max, freq_sample, FFT_SAMPLE), max_mag);
    
    // scale = fft_q15(&fx[0], &fx[FFT_SAMPLE], log_base_2(FFT_SAMPLE), 1);
    // printf("scale = %d\n", scale);

    // for (uint32_t i = 0; i < FFT_SAMPLE; i++) {
    //     printf("%ld\t\t%hd\n", i, fx[i] * (1 << scale));
    // }
    
    return (EXIT_SUCCESS);
}
#endif
