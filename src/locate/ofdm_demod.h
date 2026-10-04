/**
 * ofdm_demod.h — Internal interface for OFDM demodulation.
 *
 * This is an internal header; it is NOT installed as part of the public SDK.
 * Include locate_internal.h first (or include this from locate_internal.h).
 *
 * Implements OFDM Demodulation and ZC Synchronisation
 *   6.1  CP autocorrelation for fine symbol-start detection
 *   6.2  FFO correction from CP autocorrelation phase
 *   6.3  9-symbol (standard) or 8-symbol (legacy) extraction with LTE CP lengths
 *   6.4  1024-point FFT + 601-subcarrier extraction per symbol
 *   6.8  Sub-sample timing correction (search range [−15, +15] samples, 1000 steps)
 */

#ifndef OFDM_DEMOD_H
#define OFDM_DEMOD_H

#include "locate_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Demodulate a candidate frame (complex float32 samples at 15.36 MHz) into
 * OFDM symbols.
 *
 * Pipeline (mirrors Packet.py):
 *   1. Amplitude normalisation
 *   2. CP autocorrelation to find fine frame start and compute FFO
 *   3. FFO correction via dsp_fshift()
 *   4. Coarse symbol extraction
 *   5. Sub-sample timing correction (find_zc_offset over [-15, +15] samples)
 *   6. Re-extraction with corrected timing
 *   7. Phase correction from DC carrier (find_zc_angle)
 *   8. Final symbol extraction
 *
 * @param samples   Input complex samples at DSP_FS_TARGET (15.36 MHz)
 * @param n         Number of input samples
 * @param flags     LOCATE_FLAG_LEGACY → use 8-symbol CP table
 * @param frame_out Output frame struct (pre-allocated by caller)
 *
 * @return LOCATE_OK on success, or a negative error code.
 *
 * On success, frame_out->symbols[0..n_symbols-1] hold the subcarrier arrays,
 * frame_out->n_symbols is set, frame_out->ffo_hz and frame_out->start_sample
 * are set for diagnostics.
 */
int32_t ofdm_demodulate(const dsp_complex_t *samples, uint32_t n,
                         uint32_t flags, int do_fine, ofdm_frame_t *frame_out);

#ifdef __cplusplus
}
#endif
#endif /* OFDM_DEMOD_H */
