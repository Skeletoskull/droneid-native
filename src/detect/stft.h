/**
 * stft.h — Internal interface for STFT computation.
 *
 * Provides stft_compute(), a thin wrapper around dsp_stft_power() that
 * exposes the local detect-library STFT interface.
 *
 * The STFT uses 64-point FFT windows with 50% overlap (32-sample step),
 * matching scipy.signal.stft(nfft=64, nperseg=64) from packetizer.py.
 */

#ifndef STFT_H
#define STFT_H

#include <stdint.h>
#include "../dsp/dsp_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Compute STFT power envelope using 64-point FFT windows (50% overlap).
 *
 * power_out[k] = max(|FFT_k[f]|) over all frequency bins f
 * n_frames_out — number of STFT frames written to power_out
 *
 * power_out must be pre-allocated with at least ((n - 64) / 32 + 1) floats.
 */
void stft_compute(const dsp_complex_t *samples, uint32_t n,
                  float *power_out, uint32_t *n_frames_out);

#ifdef __cplusplus
}
#endif
#endif /* STFT_H */
