/**
 * cfo_estimator.c — CFO estimation and bandwidth validation using Welch PSD.
 *
 * Wraps dsp_estimate_cfo() with DroneID-specific bandwidth limits (8–11 MHz).
 * Mirrors estimate_offset() from helpers.py.
 *
 * Algorithm:
 *   1. Compute 2048-point Welch PSD of the candidate segment.
 *   2. Apply fftshift to centre DC  (done inside dsp_welch_psd).
 *   3. Inject a fake DC carrier at bins [1014..1034] = 1.1 × mean(PSD).
 *   4. Find all contiguous bands where PSD > mean(PSD).
 *   5. For each band, compute bandwidth = (end_bin - start_bin) * (Fs / 2048).
 *   6. Accept the first band with bandwidth in [8 MHz, 11 MHz].
 *   7. CFO = f_start - 0.5 * bandwidth  (centre of the accepted band).
 *   8. Return false (reject) if no valid band is found.
 */

#include "cfo_estimator.h"
#include "packet_detector.h"   /* PKT_BW_MIN_HZ, PKT_BW_MAX_HZ */
#include "../dsp/dsp_internal.h"

bool cfo_estimate(const dsp_complex_t *samples, uint32_t n,
                  double sample_rate, float *cfo_hz_out) {
    if (!samples || n == 0 || !cfo_hz_out)
        return false;

    return dsp_estimate_cfo(samples, n, sample_rate,
                             PKT_BW_MIN_HZ, PKT_BW_MAX_HZ,
                             cfo_hz_out);
}
