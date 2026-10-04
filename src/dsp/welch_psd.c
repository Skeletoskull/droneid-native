/**
 * welch_psd.c - Welch PSD and CFO estimation
 * Mirrors estimate_offset() from helpers.py
 */

#include "dsp_internal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define WELCH_N    DSP_WELCH_NFFT  /* 2048 */
#define WELCH_STEP (WELCH_N / 2)   /* 1024, 50% overlap */

bool dsp_welch_psd(const dsp_complex_t *samples, uint32_t n,
                   double sample_rate,
                   float *psd_out, float *freqs_out) {
    uint32_t k, seg;
    float *hann;
    float *psd_acc;
    dsp_complex_t *fft_in;
    dsp_complex_t *fft_out;
    uint32_t num_segs;
    float inv, bin_hz;

    if (n < (uint32_t)WELCH_N) return false;

    /* Heap-allocate to avoid large stack frames (~50 KB) in Release/MSVC */
    hann    = (float *)        malloc(sizeof(float)         * WELCH_N);
    psd_acc = (float *)        calloc(WELCH_N, sizeof(float));
    fft_in  = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * WELCH_N);
    fft_out = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * WELCH_N);

    if (!hann || !psd_acc || !fft_in || !fft_out) {
        free(hann); free(psd_acc); free(fft_in); free(fft_out);
        return false;
    }

    for (k = 0; k < WELCH_N; k++)
        hann[k] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * k / (WELCH_N - 1)));

    num_segs = (n - WELCH_N) / WELCH_STEP + 1;

    for (seg = 0; seg < num_segs; seg++) {
        const dsp_complex_t *p = samples + seg * WELCH_STEP;
        for (k = 0; k < WELCH_N; k++)
            fft_in[k] = _dsp_cmul(p[k], CMPLX(hann[k], 0.0f));
        dsp_fft_forward(fft_in, fft_out, WELCH_N);
        for (k = 0; k < WELCH_N; k++) {
            float mag = cabsf(fft_out[k]);
            psd_acc[k] += mag * mag;
        }
    }

    inv = 1.0f / (float)num_segs;
    bin_hz = (float)(sample_rate / WELCH_N);

    for (k = 0; k < WELCH_N; k++) {
        uint32_t s = (k + WELCH_N / 2) % WELCH_N;  /* fftshift */
        psd_out[s] = psd_acc[k] * inv;
        freqs_out[s] = (k < WELCH_N / 2) ? (float)k * bin_hz
                                           : ((float)k - (float)WELCH_N) * bin_hz;
    }

    free(hann); free(psd_acc); free(fft_in); free(fft_out);
    return true;
}

bool dsp_estimate_cfo(const dsp_complex_t *samples, uint32_t n,
                      double sample_rate,
                      float bw_min_hz, float bw_max_hz,
                      float *cfo_hz_out) {
    float *psd  = (float *)malloc(sizeof(float) * WELCH_N);
    float *freqs= (float *)malloc(sizeof(float) * WELCH_N);
    bool result = false;
    uint32_t k;
    float mean_psd, bin_hz;
    bool in_band;
    uint32_t band_start, dc;

    if (!psd || !freqs) { free(psd); free(freqs); return false; }

    /* 2048-bin Welch + edge-midpoint, matching the proven Python estimate_offset.
       NOTE: this is bin-quantized (Fs/2048); accurate enough at <=50 Msps but
       too coarse at 100 Msps — high-rate captures use the known-channel decode
       path (exact offset, no estimation) instead. */
    if (!dsp_welch_psd(samples, n, sample_rate, psd, freqs)) {
        free(psd); free(freqs);
        return false;
    }

    mean_psd = 0.0f;
    for (k = 0; k < (uint32_t)WELCH_N; k++) mean_psd += psd[k];
    mean_psd /= (float)WELCH_N;

    dc = WELCH_N / 2;                       /* inject fake DC for detection only */
    for (k = dc - 10; k < dc + 10 && k < (uint32_t)WELCH_N; k++)
        psd[k] = 1.1f * mean_psd;

    bin_hz = (float)(sample_rate / WELCH_N);
    in_band = false;
    band_start = 0;

    for (k = 0; k <= (uint32_t)WELCH_N; k++) {
        bool above = (k < (uint32_t)WELCH_N) && (psd[k] > mean_psd);
        if (above && !in_band) { band_start = k; in_band = true; }
        else if (!above && in_band) {
            uint32_t band_end = k - 1;
            float s_rel = (float)band_start - (float)(WELCH_N / 2);
            float e_rel = (float)band_end   - (float)(WELCH_N / 2);
            float bw = (e_rel - s_rel) * bin_hz;
            in_band = false;
            if (bw >= bw_min_hz && bw <= bw_max_hz) {
                *cfo_hz_out = e_rel * bin_hz - 0.5f * bw;   /* geometric midpoint */
                result = true;
                break;
            }
        }
    }

    free(psd); free(freqs);
    return result;
}
