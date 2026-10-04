/**
 * droneid_bridge_cli.c — Run a capture through droneid_process(), exactly as
 * a LabVIEW Call Library Function Node would (float64 interleaved I/Q in,
 * flat scalars out).
 *
 * Usage:
 *   droneid_bridge_cli <iq_file> <sample_rate_hz> [flags] [chunk_ms]
 *
 *   iq_file   raw interleaved little-endian float32 I/Q (converted to float64)
 *   flags     shared DETECT_/LOCATE_ flag bitfield (default 0)
 *   chunk_ms  split the capture into buffers of this length, mimicking an
 *             acquisition loop (default 0 = one call over the whole file)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "droneid_bridge.h"

static double *load_capture(const char *path, uint32_t *n_samples) {
    FILE *f = fopen(path, "rb");
    float  *tmp;
    double *iq;
    long    bytes;
    uint32_t n_floats, i;

    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    bytes = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (bytes <= 0) { fclose(f); return NULL; }

    n_floats = (uint32_t)(bytes / (long)sizeof(float)) & ~1u;
    tmp = (float *)malloc(sizeof(float) * n_floats);
    iq  = (double *)malloc(sizeof(double) * n_floats);
    if (!tmp || !iq || fread(tmp, sizeof(float), n_floats, f) != n_floats) {
        free(tmp); free(iq); fclose(f);
        return NULL;
    }
    fclose(f);
    for (i = 0; i < n_floats; i++) iq[i] = (double)tmp[i];
    free(tmp);
    *n_samples = n_floats / 2;
    return iq;
}

int main(int argc, char **argv) {
    double   fs;
    uint32_t flags, n_samples = 0, chunk, offset;
    double   chunk_ms;
    double  *iq;
    int      decoded = 0;
    uint32_t major, minor, patch;

    if (argc < 3) {
        fprintf(stderr, "usage: %s <iq_file> <sample_rate_hz> [flags] [chunk_ms]\n",
                argv[0]);
        return 2;
    }
    fs       = atof(argv[2]);
    flags    = (argc > 3) ? (uint32_t)strtoul(argv[3], NULL, 0) : 0u;
    chunk_ms = (argc > 4) ? atof(argv[4]) : 0.0;
    if (fs <= 0.0) {
        fprintf(stderr, "error: sample rate must be positive\n");
        return 2;
    }

    iq = load_capture(argv[1], &n_samples);
    if (!iq) {
        fprintf(stderr, "error: cannot read %s\n", argv[1]);
        return 1;
    }

    droneid_bridge_version(&major, &minor, &patch);
    chunk = (chunk_ms > 0.0) ? (uint32_t)(chunk_ms * 1e-3 * fs) : n_samples;
    if (chunk == 0 || chunk > n_samples) chunk = n_samples;
    printf("droneid_bridge v%u.%u.%u  file=%s  fs=%.2f Msps  flags=0x%x  "
           "%u samples (%.3f s) in buffers of %u\n",
           major, minor, patch, argv[1], fs / 1e6, flags,
           n_samples, n_samples / fs, chunk);

    for (offset = 0; offset < n_samples; offset += chunk) {
        uint32_t len = (offset + chunk > n_samples) ? n_samples - offset : chunk;
        int32_t  num_frames = 0, crc_valid = -1;
        char     serial[17];
        double   lat, lon, alt, height, app_lat, app_lon;
        int16_t  v_north, v_east;
        uint16_t seq;
        uint8_t  device_type;
        char     json[4096];
        int32_t  rc = droneid_process(iq + (size_t)offset * 2, len, fs, flags,
                                      &num_frames, &crc_valid, serial,
                                      &lat, &lon, &alt, &height, &app_lat, &app_lon,
                                      &v_north, &v_east, &seq, &device_type,
                                      json, (uint32_t)sizeof(json));

        printf("t=%8.3f s  rc=%d  frames=%d  crc=%d", offset / fs, (int)rc,
               (int)num_frames, (int)crc_valid);
        if (rc == BRIDGE_OK) {
            decoded++;
            printf("  serial=%s  seq=%u  lat=%.6f  lon=%.6f  alt=%.1f m  "
                   "pilot=(%.6f, %.6f)\n",
                   serial, (unsigned)seq, lat, lon, alt, app_lat, app_lon);
        } else {
            printf("\n");
        }
    }

    free(iq);
    printf("%d buffer(s) decoded\n", decoded);
    return decoded > 0 ? 0 : 1;
}
