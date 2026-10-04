/**
 * zc_sequence.c — Zadoff-Chu sequence generation
 *
 * Mirrors zcsequence_t() and zcsequence_f() from zcsequence.py.
 * Formula: zc[n] = exp(-j*pi*u*n*(n+1)/N), n=0..N-1
 *
 * zcsequence_f() mirrors tfft() in helpers.py:
 *   fft = np.fft.fft(sy, n=NFFT)   <- zero-pads 601 -> 1024 before FFT
 *   half_carriers = NCARRIERS//2   <- 300
 *   new_fft = np.concatenate((fft[-300:], fft[:301]))
 */

#include "dsp_internal.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dsp_zc_sequence_time(uint32_t u, uint32_t seq_length, dsp_complex_t *out) {
    uint32_t n;
    for (n = 0; n < seq_length; n++) {
        double phase = -M_PI * (double)u * (double)n * (double)(n + 1) / (double)seq_length;
        float c = (float)cos(phase);
        float s = (float)sin(phase);
        out[n] = CMPLX(c, s);
    }
}

void dsp_zc_sequence_freq(uint32_t root, dsp_complex_t *out) {
    /*
     * Mirrors zcsequence_f() / tfft() from zcsequence.py + helpers.py.
     *
     * Python tfft() calls np.fft.fft(sy, n=NFFT) which zero-pads the
     * 601-sample time-domain ZC sequence to 1024 points before FFT.
     * We must do the same here — dsp_fft_subcarriers() expects a 1024-sample
     * input; calling it with 601 samples would read past the array bounds.
     *
     * Heap-allocate to avoid large stack frames in Release/MSVC.
     */
    dsp_complex_t *time_domain = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *padded      = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NFFT);
    dsp_complex_t *full_fft    = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NFFT);

    if (!time_domain || !padded || !full_fft) {
        free(time_domain); free(padded); free(full_fft);
        memset(out, 0, sizeof(dsp_complex_t) * DSP_NCARRIERS);
        return;
    }

    dsp_zc_sequence_time(root, DSP_NCARRIERS, time_domain);

    /* Zero-pad: copy 601 samples then zero-fill remaining 423 entries */
    memcpy(padded, time_domain, sizeof(dsp_complex_t) * DSP_NCARRIERS);
    memset(padded + DSP_NCARRIERS, 0,
           sizeof(dsp_complex_t) * (DSP_NFFT - DSP_NCARRIERS));

    /* 1024-point forward FFT */
    dsp_fft_forward(padded, full_fft, DSP_NFFT);

    /*
     * Reorder to subcarrier layout matching tfft():
     *   out[0..299]   = full_fft[NFFT-300 .. NFFT-1]   (negative freqs)
     *   out[300]      = full_fft[0]                      (DC)
     *   out[301..600] = full_fft[1 .. 300]               (positive freqs)
     */
    memcpy(out,
           full_fft + DSP_NFFT - DSP_NCARRIERS_HALF,
           sizeof(dsp_complex_t) * DSP_NCARRIERS_HALF);
    out[DSP_NCARRIERS_HALF] = full_fft[0];
    memcpy(out + DSP_NCARRIERS_HALF + 1,
           full_fft + 1,
           sizeof(dsp_complex_t) * DSP_NCARRIERS_HALF);

    /* Zero the DC carrier (index 300) — mirrors zcseq_f[NCARRIERS//2] = 0 */
    out[DSP_NCARRIERS_HALF] = CMPLX(0.0f, 0.0f);

    free(time_domain);
    free(padded);
    free(full_fft);
}
