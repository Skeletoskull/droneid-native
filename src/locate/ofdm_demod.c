/**
 * ofdm_demod.c — OFDM demodulation
 *
 * Full port of Packet.py pipeline:
 *  1. Normalise amplitude
 *  2. CP autocorrelation to find fine frame start and compute FFO
 *     (mirrors find_fine_start() — scipy.signal.find_peaks, distance=1000,
 *     prominence > 1.0, take first peak)
 *  3. Apply FFO correction via dsp_fshift()
 *  4. Coarse symbol extraction
 *  5. find_zc_offset: fine sub-sample timing via RMS phase-slope minimisation
 *     (search range [-15, +15] samples, 1000 steps, always uses ZC root 600)
 *  6. Re-extract with corrected timing
 *  7. find_zc_angle: phase correction from DC carrier of ZC symbol
 *  8. Final symbol extraction with phase correction
 *
 * Key implementation note:
 *   - find_zc_offset always uses ZC root 600 (hardcoded in Python's Packet.py)
 *   - QPSK decode operates on raw (non-equalized) symbols — equalization is
 *     performed only for ZC root validation, not for the data decode path
 *
 * Responsibilities:
 *   - CP autocorrelation for fine symbol-start detection (NFFT=1024)
 *   - FFO correction: ffo = Fs/(2π·NFFT)·angle(ac[peak])
 *   - 9 symbols standard (CP=[80,72×7,80]) or 8 symbols legacy (CP=[80,72×6,80])
 *   - 1024-point FFT + 601 active subcarriers per symbol
 *   - Sub-sample timing correction over [-15, +15] samples (coarse-to-fine search)
 */

#include "locate_internal.h"
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Extract one OFDM symbol into frequency domain ───────────────────────── */

static void extract_one_symbol(const dsp_complex_t *samples, uint32_t n,
                                 int sym_start, dsp_complex_t *out) {
    if (sym_start < 0 || (uint32_t)(sym_start + DSP_NFFT) > n) {
        memset(out, 0, sizeof(dsp_complex_t) * DSP_NCARRIERS);
        return;
    }
    dsp_fft_subcarriers(samples + sym_start, out);
}

/* Extract one symbol with a fractional sample offset, interpolating ONLY the
   NFFT samples needed. Mathematically identical to applying
   dsp_with_sample_offset() over the whole candidate and then extracting the
   symbol at sym_start, but avoids resampling the entire candidate on every
   call — the dominant cost of the 1000-step find_zc_offset() search. */
static void extract_one_symbol_offset(const dsp_complex_t *samples, uint32_t n,
                                      int sym_start, float offset,
                                      dsp_complex_t *out) {
    dsp_complex_t buf[DSP_NFFT];
    int k;
    if (sym_start < 0) { memset(out, 0, sizeof(dsp_complex_t) * DSP_NCARRIERS); return; }
    for (k = 0; k < DSP_NFFT; k++) {
        double pos = (double)sym_start + (double)k + (double)offset;
        uint32_t i0, i1;
        if (pos < 0.0) pos = 0.0;
        i0 = (uint32_t)pos;
        i1 = i0 + 1;
        if (i1 >= n) {
            buf[k] = samples[n - 1];
        } else {
            float frac = (float)(pos - (double)i0);
            dsp_complex_t diff = _dsp_csub(samples[i1], samples[i0]);
            buf[k] = _dsp_cadd(samples[i0], _dsp_cmul(CMPLX(frac, 0.0f), diff));
        }
    }
    dsp_fft_subcarriers(buf, out);
}

/* ── Extract all OFDM symbols ────────────────────────────────────────────── */

static void extract_symbols(const dsp_complex_t *samples, uint32_t n,
                              int start, const int *cp_lengths, int n_syms,
                              dsp_complex_t out[][DSP_NCARRIERS]) {
    int offset = start;
    for (int s = 0; s < n_syms; s++) {
        int sym_start = offset + cp_lengths[s];
        extract_one_symbol(samples, n, sym_start, out[s]);
        offset += cp_lengths[s] + DSP_NFFT;
    }
}

/* ── find_zc_offset: fine timing via RMS phase slope minimisation ─────────── */
/*
 * Mirrors Packet.find_zc_offset().
 * IMPORTANT: zc_root is always 600 (matches Python's hardcoded seq=600 argument).
 */

/* RMS of the (unwrapped, de-meaned) phase difference between the trial-offset ZC
   symbol and the reference ZC sequence — the quantity find_zc_offset minimises.
   Scratch buffers (sym_f, adiff) are caller-owned to avoid per-eval malloc. */
static float zc_offset_rms(const dsp_complex_t *samples, uint32_t n, int sym_start,
                            float offset, const dsp_complex_t *zc_ref,
                            dsp_complex_t *sym_f, float *adiff) {
    int k;
    float mean = 0.0f, rms = 0.0f;

    extract_one_symbol_offset(samples, n, sym_start, offset, sym_f);

    for (k = 0; k < DSP_NCARRIERS; k++) {
        float mag_sym = cabsf(sym_f[k]);
        dsp_complex_t ratio;
        if (mag_sym > 1e-10f)
            ratio = _dsp_cdiv(zc_ref[k], sym_f[k]);
        else
            ratio = CMPLX(0.0f, 0.0f);
        adiff[k] = cargf(ratio);
    }
    adiff[DSP_NCARRIERS_HALF] = adiff[DSP_NCARRIERS_HALF + 1];

    for (k = 1; k < DSP_NCARRIERS; k++) {
        while (adiff[k] - adiff[k-1] >  (float)M_PI) adiff[k] -= 2.0f * (float)M_PI;
        while (adiff[k] - adiff[k-1] < -(float)M_PI) adiff[k] += 2.0f * (float)M_PI;
    }

    for (k = 0; k < DSP_NCARRIERS; k++) mean += adiff[k];
    mean /= (float)DSP_NCARRIERS;

    for (k = 0; k < DSP_NCARRIERS; k++) {
        float d = adiff[k] - mean;
        rms += d * d;
    }
    return sqrtf(rms / (float)DSP_NCARRIERS);
}

static float find_zc_offset(const dsp_complex_t *samples, uint32_t n,
                              int start, const int *cp_lengths,
                              int zc_sym_idx, uint32_t zc_root) {
    float best_rms = 1e30f;
    float best_offset = 0.0f;

    int s, i;
    int sym_start;

    /* Coarse-to-fine search over the [-15, +15] sample window. The phase-slope
       RMS vs timing offset is a smooth convex bowl, so a 0.5-sample coarse grid
       localises the minimum and a 0.025-sample fine grid around it refines —
       ~100 symbol extractions instead of the original uniform 1000, same final
       resolution. */
    const int   COARSE_N = 61;    /* step 0.5 samples over [-15, 15]      */
    const int   FINE_N   = 41;    /* step ~0.025 samples over best ± 0.5  */
    const float LO = -15.0f, HI = 15.0f;

    dsp_complex_t *zc_ref  = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    dsp_complex_t *sym_f   = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * DSP_NCARRIERS);
    float         *adiff   = (float *)        malloc(sizeof(float)          * DSP_NCARRIERS);

    if (!zc_ref || !sym_f || !adiff) {
        free(zc_ref); free(sym_f); free(adiff);
        return 0.0f;
    }

    dsp_zc_sequence_time(zc_root, DSP_NCARRIERS, zc_ref);

    /* Symbol start is independent of the trial offset — compute once. */
    sym_start = start;
    for (s = 0; s < zc_sym_idx; s++)
        sym_start += cp_lengths[s] + DSP_NFFT;
    sym_start += cp_lengths[zc_sym_idx];

    for (i = 0; i < COARSE_N; i++) {
        float offset = LO + (HI - LO) * (float)i / (float)(COARSE_N - 1);
        float rms = zc_offset_rms(samples, n, sym_start, offset, zc_ref, sym_f, adiff);
        if (rms < best_rms) { best_rms = rms; best_offset = offset; }
    }

    {
        float flo = best_offset - 0.5f, fhi = best_offset + 0.5f;
        best_rms = 1e30f;   /* re-pick within the fine window (includes coarse best) */
        for (i = 0; i < FINE_N; i++) {
            float offset = flo + (fhi - flo) * (float)i / (float)(FINE_N - 1);
            float rms = zc_offset_rms(samples, n, sym_start, offset, zc_ref, sym_f, adiff);
            if (rms < best_rms) { best_rms = rms; best_offset = offset; }
        }
    }

    free(zc_ref);
    free(sym_f);
    free(adiff);
    return best_offset;
}

/* ── find_zc_angle: phase correction from DC carrier ─────────────────────── */
/* Mirrors Packet.find_zc_angle(): returns angle of DC subcarrier (index 300). */
static float find_zc_angle(const dsp_complex_t *symbol_f) {
    return cargf(symbol_f[DSP_NCARRIERS_HALF]);
}

/* ── Public entry point ──────────────────────────────────────────────────── */

int32_t ofdm_demodulate(const dsp_complex_t *samples, uint32_t n,
                         uint32_t flags, int do_fine, ofdm_frame_t *frame_out) {
    bool legacy = (flags & LOCATE_FLAG_LEGACY) != 0;
    const int *cp_lengths = legacy ? CP_LENGTHS_LEG : CP_LENGTHS_STD;
    int n_syms  = legacy ? DSP_NSYMBOLS_LEG  : DSP_NSYMBOLS_STD;
    int zc_idx0 = legacy ? DSP_ZC_IDX_LEG_0  : DSP_ZC_IDX_STD_0;

    /* Step 1: Normalise amplitude */
    float max_amp = 0.0f;
    uint32_t i;
    for (i = 0; i < n; i++) {
        float a = cabsf(samples[i]);
        if (a > max_amp) max_amp = a;
    }
    if (max_amp < 1e-10f) return LOCATE_ERR_INVALID_ARG;

    dsp_complex_t *norm = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * n);
    if (!norm) return LOCATE_ERR_ALLOC;
    for (i = 0; i < n; i++) norm[i] = _dsp_cdiv(samples[i], CMPLX(max_amp, 0.0f));

    /* Step 2: CP autocorrelation — mirrors Python find_fine_start().
       res[k] = autocorrelation at loop variable n = NFFT+k (0-indexed from NFFT).
       scipy find_peaks() returns index k into res[], and Python uses that k
       directly as the slice index: samples[k:]. So start = k (res[] index). */
    int start = 0;
    float ffo_hz = 0.0f;
    {
        int cp_len = cp_lengths[0];
        uint32_t search_end = n - (uint32_t)cp_len;
        if (search_end <= (uint32_t)DSP_NFFT) { free(norm); return LOCATE_ERR_INVALID_ARG; }
        uint32_t res_len = search_end - (uint32_t)DSP_NFFT;

        float         *res_abs = (float *)        malloc(sizeof(float)          * res_len);
        dsp_complex_t *res_ac  = (dsp_complex_t *)malloc(sizeof(dsp_complex_t)  * res_len);
        if (!res_abs || !res_ac) { free(res_abs); free(res_ac); free(norm); return LOCATE_ERR_ALLOC; }

        for (uint32_t k = (uint32_t)DSP_NFFT; k < search_end; k++) {
            dsp_complex_t ac = CMPLX(0.0f, 0.0f);
            int j;
            for (j = 0; j < cp_len; j++) {
                dsp_complex_t cv = conjf(norm[k - DSP_NFFT + j]);
                ac = _dsp_cadd(ac, _dsp_cmul(norm[k + j], cv));
            }
            uint32_t idx = k - (uint32_t)DSP_NFFT;
            res_ac[idx]  = ac;
            res_abs[idx] = cabsf(ac);
        }

        float mean_abs = 0.0f;
        for (i = 0; i < res_len; i++) mean_abs += res_abs[i];
        mean_abs /= (float)res_len;

        /* Collect local maxima with prominence > 1.0, then apply distance=1000 filter */
        typedef struct { uint32_t pos; float val; } peak_t;
        peak_t *all_peaks = (peak_t *)malloc(sizeof(peak_t) * res_len);
        uint32_t n_peaks = 0;
        if (!all_peaks) { free(res_abs); free(res_ac); free(norm); return LOCATE_ERR_ALLOC; }

        for (i = 1; i + 1 < res_len; i++) {
            if (res_abs[i] > res_abs[i-1] && res_abs[i] > res_abs[i+1] &&
                (res_abs[i] - mean_abs) > 1.0f) {
                all_peaks[n_peaks].pos = i;
                all_peaks[n_peaks].val = res_abs[i];
                n_peaks++;
            }
        }

        uint32_t min_dist = 1000;
        for (i = 0; i < n_peaks; i++) {
            uint32_t j2;
            if (all_peaks[i].val < 0.0f) continue;
            for (j2 = i + 1; j2 < n_peaks; j2++) {
                if (all_peaks[j2].pos - all_peaks[i].pos >= min_dist) break;
                if (all_peaks[j2].val > all_peaks[i].val) { all_peaks[i].val = -1.0f; break; }
                else all_peaks[j2].val = -1.0f;
            }
        }

        int best_idx = 0;
        for (i = 0; i < n_peaks; i++) {
            if (all_peaks[i].val > 0.0f) { best_idx = (int)all_peaks[i].pos; break; }
        }
        free(all_peaks);

        if (best_idx < 0) best_idx = 0;
        if ((uint32_t)best_idx >= res_len) best_idx = (int)res_len - 1;

        ffo_hz = (float)(DSP_FS_TARGET / (2.0 * M_PI * DSP_NFFT) * cargf(res_ac[best_idx]));
        start  = best_idx;

        free(res_abs);
        free(res_ac);
    }

    /* Step 3: Apply FFO correction to norm[start:] */
    uint32_t n_from_start = n - (uint32_t)start;
    dsp_fshift(norm + start, n_from_start, -ffo_hz, DSP_FS_TARGET);

    /* Step 4: Coarse symbol extraction */
    extract_symbols(norm + start, n_from_start, 0, cp_lengths, n_syms, frame_out->symbols);

    if (!do_fine) {
        /* Coarse mode: these symbols are good enough for ZC-root validation
           during the integer-CFO search. Skip the expensive fine sub-sample
           timing (find_zc_offset, 1000 iterations) and phase steps. */
        free(norm);
        frame_out->n_symbols    = n_syms;
        frame_out->ffo_hz       = ffo_hz;
        frame_out->start_sample = start;
        return LOCATE_OK;
    }

    /* Step 5: Fine sub-sample timing correction.
       Always use ZC root 600 — matches Python's hardcoded seq=600 in find_zc_offset(). */
    float samp_offset = find_zc_offset(norm + start, n_from_start, 0, cp_lengths,
                                        zc_idx0, 600u);

    /* Step 6: Re-extract symbols with corrected timing */
    dsp_complex_t *corrected = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * n_from_start);
    if (!corrected) { free(norm); return LOCATE_ERR_ALLOC; }
    dsp_with_sample_offset(norm + start, n_from_start, samp_offset, corrected);
    extract_symbols(corrected, n_from_start, 0, cp_lengths, n_syms, frame_out->symbols);

    /* Step 7: Phase correction from DC carrier of first ZC symbol */
    float angle = find_zc_angle(frame_out->symbols[zc_idx0]);
    dsp_complex_t phase_corr = CMPLX(cosf(-angle), sinf(-angle));
    for (i = 0; i < n_from_start; i++)
        corrected[i] = _dsp_cmul(corrected[i], phase_corr);

    /* Step 8: Final symbol extraction */
    extract_symbols(corrected, n_from_start, 0, cp_lengths, n_syms, frame_out->symbols);

    free(corrected);
    free(norm);

    frame_out->n_symbols    = n_syms;
    frame_out->ffo_hz       = ffo_hz;
    frame_out->start_sample = start;
    return LOCATE_OK;
}
