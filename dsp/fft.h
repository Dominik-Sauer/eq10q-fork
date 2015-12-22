#ifndef FFT_H
#define FFT_H

#include <math.h>

#include <fftw3.h>

typedef struct {
    int index;
    int size;
    int half_size;
    double *time_samples;
    double *frequency_samples;
    fftw_plan plan;
    double normalization;
} FFT;

void initialize_FFT( FFT *fft, int fft_size, int start );
void reset_FFT( FFT *fft, int start );
void cleanup_FFT( FFT *fft );
int add_sample_and_maybe_compute_FFT( FFT* fft, double sample );
#endif
