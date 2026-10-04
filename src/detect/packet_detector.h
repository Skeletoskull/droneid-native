#ifndef PACKET_DETECTOR_H
#define PACKET_DETECTOR_H

/* Must be defined before including droneid_detect.h when building the DLL */
#ifndef DRONEID_DETECT_EXPORTS
#  define DRONEID_DETECT_EXPORTS
#endif

#include "../dsp/dsp_internal.h"
#include "../../include/droneid_detect.h"
#include <complex.h>
#include <stdlib.h>

/* Packet detection parameters */
#define PKT_MIN_DUR_STD_US   550.0f   /* µs — widened to match the proven Python detector */
#define PKT_MAX_DUR_STD_US   750.0f
#define PKT_MIN_DUR_LEG_US   565.0f
#define PKT_MAX_DUR_LEG_US   600.0f
#define PKT_GUARD_US         45.0f    /* 3 × 15 µs guard on each side — mirrors Python start/end_offset */
#define PKT_CHUNK_MS         500.0    /* Process in 500 ms chunks */
#define PKT_BW_MIN_HZ        8e6f
#define PKT_BW_MAX_HZ        11e6f
#define PKT_STFT_THRESHOLD   1.15f    /* noise floor multiplier */

/**
 * Scan a float32 IQ buffer for DroneID frame candidates.
 * Returns dynamically allocated array; caller frees with detect_free_results().
 */
int32_t packet_detect(const dsp_complex_t *samples, uint32_t n,
                      double sample_rate, uint32_t flags,
                      detection_result_t **results_out,
                      uint32_t *num_results_out);

#endif /* PACKET_DETECTOR_H */
