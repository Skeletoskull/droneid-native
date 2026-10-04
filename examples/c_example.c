/**
 * c_example.c — Standalone C example for droneid_detect + droneid_locate
 *
 * Demonstrates the typical usage pattern:
 *  1. Call detect_droneid_file() to find DroneID frames in a recorded IQ file
 *  2. For each detected frame, call locate_droneid() to decode telemetry
 *  3. Print telemetry as JSON
 *  4. Free all allocated memory
 *
 * Build: part of the project build (target c_example), or against an
 * installed SDK:
 *   gcc c_example.c -o c_example -I<sdk>/include -L<sdk>/lib \
 *       -ldroneid_detect -ldroneid_locate -Wl,-rpath,<sdk>/lib
 *
 * Run:
 *   ./c_example capture.bin 50000000 [--legacy]
 */

#include "droneid_detect.h"
#include "droneid_locate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char *argv[]) {
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <iq_file> <sample_rate_hz> [--legacy]\n",
                argv[0]);
        return 1;
    }

    const char *iq_file     = argv[1];
    double      sample_rate = atof(argv[2]);
    uint32_t    flags       = 0;
    for (int i = 3; i < argc; i++)
        if (strcmp(argv[i], "--legacy") == 0) flags |= DETECT_FLAG_LEGACY;

    /* ── Print library versions ─────────────────────────────────────────── */
    uint32_t maj, min, pat;
    detect_version(&maj, &min, &pat);
    printf("droneid_detect v%u.%u.%u\n", maj, min, pat);
    locate_version(&maj, &min, &pat);
    printf("droneid_locate v%u.%u.%u\n\n", maj, min, pat);

    /* ── Step 1: Detect frames ──────────────────────────────────────────── */
    detection_result_t *det = NULL;
    uint32_t n_det = 0;

    int32_t rc = detect_droneid_file(iq_file, sample_rate, flags, &det, &n_det);
    if (rc == DETECT_STATUS_NO_FRAMES || n_det == 0) {
        printf("No DroneID frames detected in %s\n", iq_file);
        return 0;
    }
    if (rc != DETECT_OK) {
        fprintf(stderr, "detect_droneid_file error: %d\n", rc);
        return 1;
    }
    printf("Detected %u frame(s)\n\n", n_det);

    /* ── Step 2: Decode each frame ──────────────────────────────────────── */
    for (uint32_t i = 0; i < n_det; i++) {
        printf("=== Frame %u ===\n", i);
        printf("  Start : %.6f s\n", det[i].start_time_s);
        printf("  Length: %.1f us\n", det[i].duration_s * 1e6);
        printf("  CFO   : %.0f Hz\n", det[i].cfo_hz);

        telemetry_result_t tel;
        rc = locate_droneid(det[i].candidate_samples,
                             det[i].num_candidate_samples,
                             flags, &tel);
        if (rc != LOCATE_OK) {
            printf("  Decode failed (code %d)\n\n", rc);
            continue;
        }

        /* Print as JSON */
        char json[2048];
        telemetry_to_json(&tel, json, sizeof(json));
        printf("%s\n\n", json);

        /* Example: access individual fields */
        printf("  Drone : lat=%.6f  lon=%.6f  alt=%.1f m\n",
               tel.latitude, tel.longitude, tel.altitude_m);
        printf("  Serial: %s\n", tel.serial_number);
        printf("  CRC   : %s\n\n", tel.crc_valid ? "valid" : "INVALID");
    }

    /* ── Step 3: Free memory ────────────────────────────────────────────── */
    detect_free_results(det, n_det);
    return 0;
}
