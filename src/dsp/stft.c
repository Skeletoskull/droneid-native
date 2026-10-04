/**
 * stft.c - STFT power envelope computation
 * Mirrors scipy.signal.stft(nfft=64, nperseg=64) from packetizer.py.
 * Uses 50% overlap (step=32), rectangular window.
 */

#include "dsp_internal.h"
#include <math.h>
#include <stdlib.h>
#ifdef _OPENMP
#  include <omp.h>
#endif

void dsp_stft_power(const dsp_complex_t *samples, uint32_t n,
                    float *power_out, uint32_t *n_frames) {
    uint32_t step = DSP_STFT_NFFT / 2;
    uint32_t frames = (n >= (uint32_t)DSP_STFT_NFFT) ?
                      (n - (uint32_t)DSP_STFT_NFFT) / step + 1 : 0;
    int n_plans, t, ok;
    long f;
    dsp_fft_plan **plans;

    *n_frames = frames;
    if (frames == 0) return;

    /* The envelope runs millions of fixed-size FFTs, each frame independent —
       the dominant cost of detection at high sample rates. Parallelise across
       cores with one plan per thread (a plan owns its own FFT state, so exec()
       is lock-free; KissFFT is not guaranteed reentrant on a *shared* cfg, hence
       per-thread). Per frame: feed straight from the input (no copy — the FFT
       reads it read-only) and take a single sqrt — argmax(|X|) == argmax(|X|^2),
       so track the max magnitude-squared and sqrt only the winner. Output is the
       same max-bin magnitude as the original scalar path. */
#ifdef _OPENMP
    n_plans = omp_get_max_threads();
    if (n_plans < 1) n_plans = 1;
#else
    n_plans = 1;
#endif

    plans = (dsp_fft_plan **)malloc(sizeof(*plans) * (size_t)n_plans);
    ok = (plans != NULL);
    for (t = 0; ok && t < n_plans; t++) {
        plans[t] = dsp_fft_plan_create((uint32_t)DSP_STFT_NFFT, 0);
        if (!plans[t]) ok = 0;
    }

    if (!ok) {
        /* allocation failure → single-thread fallback via the locked per-call path */
        dsp_complex_t fft_in[DSP_STFT_NFFT], fft_out[DSP_STFT_NFFT];
        uint32_t k;
        if (plans) {
            for (t = 0; t < n_plans; t++) dsp_fft_plan_destroy(plans[t]);
            free(plans);
        }
        for (f = 0; f < (long)frames; f++) {
            float max_mag = 0.0f;
            for (k = 0; k < DSP_STFT_NFFT; k++) fft_in[k] = samples[(size_t)f * step + k];
            dsp_fft_forward(fft_in, fft_out, DSP_STFT_NFFT);
            for (k = 0; k < DSP_STFT_NFFT; k++) {
                float mag = cabsf(fft_out[k]);
                if (mag > max_mag) max_mag = mag;
            }
            power_out[f] = max_mag;
        }
        return;
    }

#ifdef _OPENMP
#  pragma omp parallel for schedule(static)
#endif
    for (f = 0; f < (long)frames; f++) {
        dsp_complex_t fft_out[DSP_STFT_NFFT];
        float max_mag2 = 0.0f;
        uint32_t k;
        int tid = 0;
#ifdef _OPENMP
        tid = omp_get_thread_num();
#endif
        dsp_fft_plan_exec(plans[tid], samples + (size_t)f * step, fft_out);
        for (k = 0; k < DSP_STFT_NFFT; k++) {
            float re = crealf(fft_out[k]);
            float im = cimagf(fft_out[k]);
            float m2 = re * re + im * im;
            if (m2 > max_mag2) max_mag2 = m2;
        }
        power_out[f] = sqrtf(max_mag2);
    }

    for (t = 0; t < n_plans; t++) dsp_fft_plan_destroy(plans[t]);
    free(plans);
}

float dsp_stft_mean_magnitude(const dsp_complex_t *samples, uint32_t n) {
    uint32_t step = DSP_STFT_NFFT / 2;
    uint32_t frames = (n >= (uint32_t)DSP_STFT_NFFT) ?
                      (n - (uint32_t)DSP_STFT_NFFT) / step + 1 : 0;
    if (frames == 0) return 0.0f;

    dsp_complex_t fft_in[DSP_STFT_NFFT];
    dsp_complex_t fft_out[DSP_STFT_NFFT];
    double sum = 0.0;
    uint64_t count = 0;

    for (uint32_t f = 0; f < frames; f++) {
        uint32_t k;
        for (k = 0; k < DSP_STFT_NFFT; k++)
            fft_in[k] = samples[f * step + k];
        dsp_fft_forward(fft_in, fft_out, DSP_STFT_NFFT);
        for (k = 0; k < DSP_STFT_NFFT; k++) {
            sum += (double)cabsf(fft_out[k]);
            count++;
        }
    }
    return (float)(sum / (double)count);
}
