/**
 * turbo_decode.h — soft-decision + LTE turbo FEC decode (DJI DroneID).
 *
 * Recovers beacons that the systematic-only hard decoder cannot, by using
 * all three turbo streams and running an LTE max-log-MAP turbo decoder.
 * Standard 9-symbol frames only (legacy returns LOCATE_ERR_CRC_FAIL).
 */
#ifndef TURBO_DECODE_H
#define TURBO_DECODE_H

#include "locate_internal.h"

/**
 * Soft-decision + turbo FEC decode of the data symbols (brute-forces the 4
 * QPSK phase rotations). On a valid CRC-16, fills payload_out with the
 * decoded DUML bytes and returns LOCATE_OK; otherwise LOCATE_ERR_CRC_FAIL.
 */
int32_t turbo_decode_dji(dsp_complex_t syms[][DSP_NCARRIERS],
                         int n_data_symbols, bool legacy,
                         uint8_t *payload_out);

#endif /* TURBO_DECODE_H */
