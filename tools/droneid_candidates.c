/**
 * droneid_candidates.c — List the DroneID burst candidates in a capture.
 *
 * Runs only the detection stage and prints each candidate's start time,
 * duration and carrier frequency offset (relative to the capture centre).
 * Useful to check that the capture format and sample rate are right before
 * looking at decoding.
 *
 * Usage:
 *   droneid_candidates <iq_file> <sample_rate_hz> [flags]
 *
 *   flags  DETECT_FLAG_* bitfield (default 0), e.g. 0x1 = legacy, 0x4 = conjugate
 */
#include <stdio.h>
#include <stdlib.h>
#include "droneid_detect.h"

int main(int argc, char **argv) {
    detection_result_t *res = NULL;
    uint32_t n = 0, i;
    double fs;
    uint32_t flags;
    int32_t rc;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <iq_file> <sample_rate_hz> [flags]\n", argv[0]);
        return 2;
    }
    fs    = atof(argv[2]);
    flags = (argc > 3) ? (uint32_t)strtoul(argv[3], NULL, 0) : 0u;
    if (fs <= 0.0) {
        fprintf(stderr, "error: sample rate must be positive\n");
        return 2;
    }

    rc = detect_droneid_file(argv[1], fs, flags, &res, &n);
    printf("%s @ %.2f Msps: rc=%d, %u candidate(s)\n", argv[1], fs / 1e6, (int)rc, n);
    for (i = 0; i < n; i++)
        printf("  [%u] t=%.6f s  dur=%.1f us  cfo=%+.3f MHz\n",
               i, res[i].start_time_s, res[i].duration_s * 1e6,
               res[i].cfo_hz / 1e6);

    detect_free_results(res, n);
    return (rc < 0) ? 1 : 0;
}
