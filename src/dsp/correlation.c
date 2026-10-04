/**
 * correlation.c — Cross-correlation and argmax utilities
 *
 * Mirrors corr() from helpers.py (one-sided cross-correlation).
 */

#include "dsp_internal.h"
#include <math.h>
#include <string.h>

void dsp_correlate(const dsp_complex_t *x, const dsp_complex_t *y,
                   dsp_complex_t *out, uint32_t n) {
    for (uint32_t k = 0; k < n; k++) {
        dsp_complex_t acc = CMPLX(0.0f, 0.0f);
        for (uint32_t i = 0; i < n - k; i++) {
            dsp_complex_t conj_y = conjf(y[i]);
            dsp_complex_t prod = _dsp_cmul(x[i + k], conj_y);
            acc = _dsp_cadd(acc, prod);
        }
        out[k] = acc;
    }
}

uint32_t dsp_argmax_abs(const dsp_complex_t *x, uint32_t n) {
    uint32_t best = 0;
    float best_val = cabsf(x[0]);
    for (uint32_t i = 1; i < n; i++) {
        float v = cabsf(x[i]);
        if (v > best_val) {
            best_val = v;
            best = i;
        }
    }
    return best;
}
