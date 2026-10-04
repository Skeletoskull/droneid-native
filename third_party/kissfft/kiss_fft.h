#ifndef KISS_FFT_H
#define KISS_FFT_H

/*
 * KissFFT — minimal single-precision complex FFT
 * BSD-3-Clause License
 * https://github.com/mborgerding/kissfft
 *
 * This is a self-contained copy of KissFFT for use as a fallback
 * when FFTW3 is not available.
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float r;
    float i;
} kiss_fft_cpx;

typedef struct kiss_fft_state *kiss_fft_cfg;

/**
 * Allocate a KissFFT plan.
 * @param nfft      FFT size (must be a power of 2 for best performance)
 * @param inverse_fft  0 for forward FFT, 1 for inverse FFT
 * @param mem       If non-NULL, use this pre-allocated memory
 * @param lenmem    Size of pre-allocated memory; set to required size if mem==NULL
 */
kiss_fft_cfg kiss_fft_alloc(int nfft, int inverse_fft, void *mem, size_t *lenmem);

/**
 * Perform the FFT.
 * @param cfg   Plan allocated by kiss_fft_alloc
 * @param fin   Input array of nfft complex samples
 * @param fout  Output array of nfft complex samples
 */
void kiss_fft(kiss_fft_cfg cfg, const kiss_fft_cpx *fin, kiss_fft_cpx *fout);

/**
 * Free a KissFFT plan allocated with kiss_fft_alloc.
 */
void kiss_fft_free(kiss_fft_cfg cfg);

/**
 * Cleanup any global memory (call at program exit).
 */
void kiss_fft_cleanup(void);

#ifdef __cplusplus
}
#endif
#endif /* KISS_FFT_H */
