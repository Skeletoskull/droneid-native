/**
 * qpsk_decode.h — QPSK demodulation, Gold descrambling, and turbo de-rate-match
 *
 * Internal interface for the QPSK decode stage of the DroneID location pipeline.
 *
 * Pipeline position:
 *   ofdm_demod → zc_sync → [qpsk_decode] → duml_parser
 *
 * This module implements:
 *   - QPSK quadrant decision with four phase rotation tables (0°, 90°, 180°, 270°)
 *   - Brute-force phase rotation search with CRC validation
 *   - DC subcarrier removal (index 300 of 601 subcarriers)
 *   - Gold sequence descrambling (seed 0x12345678, Nc=1600)
 *   - Systematic stream extraction from cyclic buffer at offset 4148
 *   - 3GPP turbo de-rate-match (32-column permutation)
 */

#ifndef QPSK_DECODE_H
#define QPSK_DECODE_H

#include "locate_internal.h"

/**
 * Demodulate QPSK symbols, descramble, and recover the DUML payload bytes.
 *
 * Tries all four QPSK phase rotations (0°, 90°, 180°, 270°) and returns the
 * decoded payload for the first rotation that produces a valid CRC-16.
 *
 * @param equalized       Array of n_data_symbols rows, each DSP_NCARRIERS
 *                        (601) equalised complex subcarriers, as produced by
 *                        zc_sync_and_equalize().
 * @param n_data_symbols  Number of data-bearing OFDM symbols (7 standard, 6 legacy).
 * @param legacy          true to use legacy (6-symbol) data extraction path.
 * @param payload_out     Caller-supplied buffer of at least DUML_PAYLOAD_LEN (91)
 *                        bytes to receive the decoded DUML payload.
 *
 * @return LOCATE_OK if a valid CRC was found and payload_out is populated.
 *         LOCATE_ERR_CRC_FAIL if no phase rotation produced a valid CRC.
 *         LOCATE_ERR_ALLOC if a heap allocation failed.
 */
int32_t qpsk_decode(dsp_complex_t equalized[][DSP_NCARRIERS],
                    int n_data_symbols, bool legacy,
                    uint8_t *payload_out);

#endif /* QPSK_DECODE_H */
