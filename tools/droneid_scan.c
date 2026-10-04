/**
 * droneid_scan.c — Decode every DroneID frame in a recorded capture.
 *
 * Uses locate_droneid_file(), i.e. detection + decoding in one call.
 *
 * Usage:
 *   droneid_scan <iq_file> <sample_rate_hz> [flags]
 *
 *   iq_file         raw interleaved little-endian float32 I/Q
 *   sample_rate_hz  capture sample rate, e.g. 50000000
 *   flags           LOCATE_FLAG_* bitfield (default 0), e.g. 0x1 = legacy
 */
#include <stdio.h>
#include <stdlib.h>
#include "droneid_locate.h"

int main(int argc, char **argv) {
    telemetry_result_t *res = NULL;
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

    rc = locate_droneid_file(argv[1], fs, flags, &res, &n);
    printf("%s @ %.2f Msps: rc=%d, %u frame(s)\n", argv[1], fs / 1e6, (int)rc, n);

    for (i = 0; i < n; i++)
        printf("  [%u] serial=%s  seq=%u  lat=%.6f  lon=%.6f  alt=%.1f m  "
               "pilot=(%.6f, %.6f)  crc=%s\n",
               i, res[i].serial_number, (unsigned)res[i].sequence_number,
               res[i].latitude, res[i].longitude, res[i].altitude_m,
               res[i].app_lat, res[i].app_lon, res[i].crc_valid ? "OK" : "FAIL");

    locate_free_results(res, n);
    return (rc == LOCATE_OK && n > 0) ? 0 : 1;
}
