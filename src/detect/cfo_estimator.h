/**
 * cfo_estimator.h — Internal interface for CFO estimation.
 *
 * Wraps dsp_estimate_cfo() with DroneID-specific bandwidth limits (8–11 MHz).
 * Mirrors estimate_offset() from helpers.py.
 */

#ifndef CFO_ESTIMATOR_H
#define CFO_ESTIMATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "../dsp/dsp_internal.h"  /* dsp_complex_t */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Estimate carrier frequency offset and validate signal bandwidth.
 *
 * Uses Welch PSD with a 2048-point FFT to detect the signal band.
 * Rejects candidates whose bandwidth falls outside 8 MHz–11 MHz.
 *
 * @param samples     Input complex samples at the SDR capture rate
 * @param n           Number of samples
 * @param sample_rate SDR capture sample rate in Hz
 * @param cfo_hz_out  Estimated CFO in Hz (written only on return true)
 *
 * @return true if a valid DroneID band is found; false to reject the candidate
 */
bool cfo_estimate(const dsp_complex_t *samples, uint32_t n,
                  double sample_rate, float *cfo_hz_out);

#ifdef __cplusplus
}
#endif
#endif /* CFO_ESTIMATOR_H */
