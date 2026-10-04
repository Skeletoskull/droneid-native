/**
 * resample.c — Anti-aliased (Lanczos) resampler + sub-sample timing offset.
 *
 * The DroneID decode path resamples a captured channel down to 15.36 MHz. When
 * the capture rate is high (e.g. 100 Msps → 6.5x decimation) in a crowded band,
 * plain linear interpolation barely filters, so strong out-of-band signals
 * (WiFi/BT) ALIAS onto the DroneID and destroy it. A windowed-sinc (Lanczos-3)
 * kernel whose cutoff tracks the OUTPUT Nyquist provides real anti-aliasing,
 * equivalent in intent to scipy.signal.resample_poly.
 */

#include "dsp_internal.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double rs_sinc(double x) {
    if (x == 0.0) return 1.0;
    double px = M_PI * x;
    return sin(px) / px;
}

uint32_t dsp_resample_output_length(uint32_t n_in, double fs_in, double fs_out) {
    return (uint32_t)floor((double)n_in * fs_out / fs_in);
}

uint32_t dsp_resample(const dsp_complex_t *in, uint32_t n_in,
                      dsp_complex_t *out, uint32_t n_out,
                      double fs_in, double fs_out) {
    const int A = 3;                              /* Lanczos-3 lobes */
    double ratio = fs_in / fs_out;                /* input samples per output sample */
    double scale = (ratio > 1.0) ? ratio : 1.0;   /* stretch kernel when decimating → LPF at fs_out/2 */
    double half  = (double)A * scale;
    uint32_t k;

    if (n_in == 0) return 0;

    for (k = 0; k < n_out; k++) {
        double pos = (double)k * ratio;
        long jmin = (long)ceil(pos - half);
        long jmax = (long)floor(pos + half);
        double sre = 0.0, sim = 0.0, wsum = 0.0;
        long j;

        if (jmin < 0) jmin = 0;
        if (jmax > (long)n_in - 1) jmax = (long)n_in - 1;

        for (j = jmin; j <= jmax; j++) {
            double x = ((double)j - pos) / scale;     /* output-normalised distance */
            double w;
            if (x <= -A || x >= A) continue;
            w = rs_sinc(x) * rs_sinc(x / (double)A);   /* sinc LPF × Lanczos window */
            sre  += (double)crealf(in[j]) * w;
            sim  += (double)cimagf(in[j]) * w;
            wsum += w;
        }
        if (wsum != 0.0)
            out[k] = CMPLX((float)(sre / wsum), (float)(sim / wsum));
        else
            out[k] = in[(jmin < (long)n_in) ? jmin : (long)n_in - 1];
    }
    return n_out;
}

/**
 * Apply sub-sample timing offset using linear interpolation.
 * Equivalent to helpers.py with_sample_offset().
 *
 * Produces n output samples starting at fractional position `offset`.
 */
void dsp_with_sample_offset(const dsp_complex_t *in, uint32_t n,
                             float offset, dsp_complex_t *out) {
    for (uint32_t k = 0; k < n; k++) {
        double pos = (double)offset + (double)k;
        if (pos < 0.0) pos = 0.0;
        uint32_t idx0 = (uint32_t)pos;
        uint32_t idx1 = idx0 + 1;
        if (idx1 >= n) {
            out[k] = in[n - 1];
        } else {
            float frac = (float)(pos - (double)idx0);
            dsp_complex_t s0 = in[idx0];
            dsp_complex_t s1 = in[idx1];
            dsp_complex_t diff = _dsp_csub(s1, s0);
            out[k] = _dsp_cadd(s0, _dsp_cmul(CMPLX(frac, 0.0f), diff));
        }
    }
}
