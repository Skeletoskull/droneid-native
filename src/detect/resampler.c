/**
 * resampler.c — Resampling to 15.36 MHz for the detection library.
 *
 * Thin wrapper around dsp_resample() that allocates the output buffer and
 * targets DRONEID_SAMPLE_RATE_HZ (15.36 MHz).
 *
 * Algorithm: linear interpolation at positions k * (fs_in / fs_out)
 * for k = 0 … n_out-1, where n_out = floor(n_in * fs_out / fs_in).
 * (Mirrors resample() from helpers.py.)
 */

#include "resampler.h"
#include "../dsp/dsp_internal.h"
#include <stdlib.h>

uint32_t detect_resample_to_15m36(const dsp_complex_t *in, uint32_t n_in,
                                   double fs_in,
                                   dsp_complex_t **out_ptr) {
    if (!in || n_in == 0 || fs_in <= 0.0 || !out_ptr)
        return 0;

    *out_ptr = NULL;

    uint32_t n_out = dsp_resample_output_length(n_in, fs_in, DRONEID_SAMPLE_RATE_HZ);
    if (n_out == 0)
        return 0;

    dsp_complex_t *buf = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * n_out);
    if (!buf)
        return 0;

    uint32_t written = dsp_resample(in, n_in, buf, n_out, fs_in, DRONEID_SAMPLE_RATE_HZ);
    if (written == 0) {
        free(buf);
        return 0;
    }

    *out_ptr = buf;
    return written;
}
