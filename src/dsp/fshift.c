/**
 * fshift.c — Frequency shift utility
 *
 * Mirrors fshift() from helpers.py.
 * samples[k] *= exp(j * 2*pi * offset_hz * k / sample_rate)
 */

#include "dsp_internal.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void dsp_fshift(dsp_complex_t *samples, uint32_t n,
                float offset_hz, double sample_rate) {
    /* Mirror Python: x = np.linspace(0, N/Fs, N)
       x[k] = k * (N/Fs) / (N-1) = k * N / (Fs * (N-1)) */
    double end_time = (double)n / sample_rate;
    double step = (n > 1) ? end_time / (double)(n - 1) : 0.0;
    for (uint32_t k = 0; k < n; k++) {
        double phase = 2.0 * M_PI * (double)offset_hz * step * (double)k;
        float c = (float)cos(phase);
        float s = (float)sin(phase);
        dsp_complex_t rot = CMPLX(c, s);
        samples[k] = _dsp_cmul(samples[k], rot);
    }
}
