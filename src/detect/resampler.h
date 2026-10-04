/**
 * resampler.h — Internal interface for resampling to 15.36 MHz.
 *
 * Exposes a detect-layer convenience wrapper around dsp_resample() that
 * allocates the output buffer and targets DRONEID_SAMPLE_RATE_HZ.
 */

#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <stdint.h>
#include "../dsp/dsp_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Target sample rate for the DroneID decoder (15.36 MHz). */
#define DRONEID_SAMPLE_RATE_HZ 15360000.0

/**
 * Resample candidate samples to DRONEID_SAMPLE_RATE_HZ (15.36 MHz).
 *
 * Allocates an output buffer of exactly
 *   floor(n_in * DRONEID_SAMPLE_RATE_HZ / fs_in) samples
 * via malloc() and stores the pointer in *out_ptr.  The caller is
 * responsible for calling free(*out_ptr) when done.
 *
 * Returns the number of output samples written, or 0 on error
 * (invalid arguments, allocation failure, or zero-length output).
 *
 * Uses linear interpolation (delegates to dsp_resample()).
 */
uint32_t detect_resample_to_15m36(const dsp_complex_t *in, uint32_t n_in,
                                   double fs_in,
                                   dsp_complex_t **out_ptr);

#ifdef __cplusplus
}
#endif
#endif /* RESAMPLER_H */
