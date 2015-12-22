#include "fft.h"
#include <math.h>

#ifndef M_PI
    #define M_PI (3.14159265358979323846)
#endif

static inline double _apply_window( double sample, int index, int size );
static inline void _compute_FFT ( FFT* fft, FFT* delayed_fft );
static inline void _normalize_frequency_sample( FFT* fft, FFT* delayed_fft, int i );

void initialize_FFT( FFT *fft, int size, int start ) {
    fft->size = size;
    fft->half_size = size / 2;
    fft->time_samples = (double*) fftw_malloc(sizeof(double) * size);
    fft->frequency_samples = (double*) fftw_malloc(sizeof(double) * size);
    fft->plan = fftw_plan_r2r_1d( size, fft->time_samples, fft->frequency_samples, FFTW_R2HC, FFTW_ESTIMATE );
    fft->normalization = pow(2.0/ ((double) size), 2.0);

    reset_FFT( fft, start );
}

void reset_FFT( FFT *fft, int start ) {
    for( int i = 0; i <= fft->size; i++ ) {
        fft->time_samples[i] = fft->frequency_samples[i] = 0.0;
    }

    fft->index = start;
}

void cleanup_FFT( FFT *fft ) {
    fftw_destroy_plan(fft->plan);
    fftw_free(fft->time_samples);
    fftw_free(fft->frequency_samples);
}

int add_sample_and_maybe_compute_FFT( FFT* fft, double sample, FFT* delayed_fft ){
    fft->time_samples[fft->index] = _apply_window( sample, fft->index, fft->size );

    if( ++fft->index < fft->size )
        return 0;

    _compute_FFT( fft, delayed_fft );
    fft->index = 0;

    return 1;
}

static inline double _apply_window( double sample, int index, int size ) {
    //Hanning Windowing
    return sample * 0.5 * (
        1.0 - cos(
            ( 2.0 * M_PI * (double)index )
            /
            (double)(size - 1)
        )
    );
}

static inline void _compute_FFT( FFT* fft, FFT* delayed_fft ) {
    fftw_execute( fft->plan );
    for( int i = 0; i <= fft->half_size; i++ )
        _normalize_frequency_sample( fft, delayed_fft, i );
}

static inline void _normalize_frequency_sample( FFT* fft, FFT* delayed_fft, int i ) {
    double real, img;

    real = fft->frequency_samples[i];

    img = ( i > 0 && i < fft->half_size )
        ? fft->frequency_samples[ fft->size - i ]
        : 0.0;

    fft->frequency_samples[i] = fft->normalization * ( real*real + img*img );

    if ( delayed_fft ) {
        fft->frequency_samples[i] += delayed_fft->frequency_samples[i];
        fft->frequency_samples[i] /= 2.0;
    }
}
