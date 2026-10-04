/**
 * zc_sync.h — Internal interface for ZC synchronisation, channel estimation,
 *              and zero-forcing equalisation.
 *
 * This is an internal header; it is NOT installed as part of the public SDK.
 * Include locate_internal.h first (it pulls in ofdm_demod.h and defines
 * ofdm_frame_t).
 *
 * Responsibilities:
 *   - locate_droneid() accepts candidate frame at 15.36 MHz
 *   - Stateless API — no context management required
 *   - Standard and legacy mode support
 *   - ZC root detection in symbols 3&5 (standard) or 2&4 (legacy),
 *     correlating against all candidate roots 1–600
 *   - Second ZC root validation: must be 147 in standard mode;
 *     return LOCATE_ERR_ZC_NOT_FOUND if not
 *   - Channel estimation (zero-forcing) using both ZC symbols;
 *     average of the two per-subcarrier channel estimates
 *   - Zero-forcing equalisation applied to all data-bearing symbols
 */

#ifndef ZC_SYNC_H
#define ZC_SYNC_H

#include "locate_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Perform ZC root detection, channel estimation, and zero-forcing equalisation.
 *
 * Pipeline (mirrors Packet.py find_zc_seq(), estimate_channel(), symbol_equalized()):
 *
 *  1. Detect ZC root index in first ZC symbol (idx 3 standard, idx 2 legacy)
 *     by correlating received subcarriers against ZC time-domain sequences for
 *     all candidate roots r = 1..600. best_root = argmax(max|corr(sym, zc_r)|) + 1
 *
 *  2. Detect ZC root index in second ZC symbol (idx 5 standard, idx 4 legacy)
 *     using the same correlation procedure.
 *
 *  3. Validate: in standard mode, second ZC root MUST be 147. If not, return
 *     LOCATE_ERR_ZC_NOT_FOUND.
 *
 *  4. Channel estimation (zero-forcing) for each ZC symbol:
 *       zc_freq = dsp_zc_sequence_freq(root, 601)
 *       zc_freq[300] = 1  (restore DC)
 *       channel_k = received_k / zc_freq_k   (element-wise complex division)
 *     Average the two channel estimates: channel = (ch0 + ch1) / 2
 *
 *  5. Apply zero-forcing equalisation to all data-bearing symbols:
 *       equalized[di][k] = symbol_f[k] / channel[k]
 *
 * @param frame              OFDM frame produced by ofdm_demodulate()
 * @param flags              LOCATE_FLAG_LEGACY → use 8-symbol table, ZC idx [2,4]
 * @param equalized          Output array [n_data_symbols][DSP_NCARRIERS];
 *                           caller pre-allocates. Populated on LOCATE_OK.
 * @param n_data_symbols_out Receives the number of populated data-symbol rows.
 *
 * @return LOCATE_OK on success
 *         LOCATE_ERR_ZC_NOT_FOUND if second ZC root ≠ 147 (standard mode)
 *         LOCATE_ERR_ALLOC if internal heap allocation fails
 */
int32_t zc_sync_and_equalize(ofdm_frame_t *frame, uint32_t flags,
                               dsp_complex_t equalized[][DSP_NCARRIERS],
                               int *n_data_symbols_out);

#ifdef __cplusplus
}
#endif
#endif /* ZC_SYNC_H */
