/**
 * stft.c — Short-Time Fourier Transform (64-point, 50% overlap).
 *
 * Thin wrapper around dsp_stft_power() that provides the stft_compute()
 * interface declared in stft.h.
 *
 * Mirrors find_packet_candidate_time() (STFT step) from packetizer.py:
 *   scipy.signal.stft(x, fs, nfft=64, nperseg=64) → 50% overlap, step=32.
 *
 * power_out[k] = max(|FFT_k[f]|) over all frequency bins f.
 */

#include "stft.h"
#include "../dsp/dsp_internal.h"
#include <stdlib.h>

/**
 * stft_compute - Compute STFT power envelope.
 *
 * @samples       - Input complex IQ samples
 * @n             - Number of samples
 * @power_out     - Pre-allocated output array (at least ceil(n/32) floats)
 * @n_frames_out  - Number of STFT frames written to power_out
 *
 * For each 64-sample window (with 50% overlap / 32-sample step):
 *   power_out[k] = max(|FFT_k[f]|) over all 64 frequency bins f
 *
 * This matches scipy.signal.stft(x, nfft=64, nperseg=64, noverlap=32).
 */
void stft_compute(const dsp_complex_t *samples, uint32_t n,
                  float *power_out, uint32_t *n_frames_out) {
    dsp_stft_power(samples, n, power_out, n_frames_out);
}
