/**
 * test_bridge.c — Unit tests for droneid_bridge (droneid_process)
 *
 * Exercises argument validation, output zeroing and the no-signal path with
 * synthetic buffers. No capture file is required.
 */
#include "droneid_bridge.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int g_pass = 0, g_fail = 0;

#define ASSERT(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", (msg)); g_pass++; } \
    else      { printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__); g_fail++; } \
} while (0)

typedef struct {
    int32_t  num_frames, crc_valid;
    char     serial[17];
    double   lat, lon, alt, height, app_lat, app_lon;
    int16_t  v_north, v_east;
    uint16_t seq;
    uint8_t  device_type;
    char     json[4096];
} bridge_outputs_t;

static void poison(bridge_outputs_t *o) {
    memset(o, 0x5A, sizeof(*o));
}

static int32_t call_process(const double *iq, uint32_t n, double fs,
                            uint32_t flags, bridge_outputs_t *o) {
    return droneid_process(iq, n, fs, flags,
                           &o->num_frames, &o->crc_valid, o->serial,
                           &o->lat, &o->lon, &o->alt, &o->height,
                           &o->app_lat, &o->app_lon,
                           &o->v_north, &o->v_east, &o->seq, &o->device_type,
                           o->json, (uint32_t)sizeof(o->json));
}

/* Deterministic LCG so the test is reproducible on every platform. */
static double lcg_uniform(uint32_t *state) {
    *state = *state * 1664525u + 1013904223u;
    return ((double)(*state >> 8) / 16777216.0) - 0.5;
}

static void test_version(void) {
    uint32_t major = 99, minor = 99, patch = 99;
    printf("[test_version]\n");
    droneid_bridge_version(&major, &minor, &patch);
    ASSERT(major == 1 && minor == 0, "bridge reports version 1.0.x");
    droneid_bridge_version(NULL, NULL, NULL);
    ASSERT(1, "NULL version pointers are accepted");
}

static void test_invalid_args(void) {
    bridge_outputs_t o;
    double iq[4] = { 0 };
    printf("[test_invalid_args]\n");
    ASSERT(call_process(NULL, 2, 50e6, 0, &o) == BRIDGE_ERR_INVALID_ARG,
           "NULL IQ buffer returns BRIDGE_ERR_INVALID_ARG");
    ASSERT(call_process(iq, 0, 50e6, 0, &o) == BRIDGE_ERR_INVALID_ARG,
           "zero samples returns BRIDGE_ERR_INVALID_ARG");
}

static void test_noise_returns_no_frames(void) {
    const double fs = 50e6;
    const uint32_t n = (uint32_t)(0.05 * fs);   /* 50 ms */
    uint32_t i, seed = 12345u;
    double *iq = (double *)malloc(sizeof(double) * 2u * n);
    bridge_outputs_t o;
    int32_t rc;

    printf("[test_noise_returns_no_frames]\n");
    if (!iq) { ASSERT(0, "allocate noise buffer"); return; }
    for (i = 0; i < 2u * n; i++) iq[i] = 0.01 * lcg_uniform(&seed);

    poison(&o);
    rc = call_process(iq, n, fs, 0, &o);
    ASSERT(rc == BRIDGE_NO_FRAMES, "noise-only buffer returns BRIDGE_NO_FRAMES");
    ASSERT(o.num_frames == 0, "num_frames_out is 0");
    ASSERT(o.crc_valid == -1, "crc_valid_out is -1 when nothing was detected");
    ASSERT(o.serial[0] == '\0', "serial_out is cleared");
    ASSERT(o.json[0] == '\0', "json_out is cleared");
    ASSERT(o.lat == 0.0 && o.lon == 0.0, "position outputs are zeroed");
    free(iq);
}

static void test_zero_signal(void) {
    const double fs = 100e6;
    const uint32_t n = (uint32_t)(0.02 * fs);   /* 20 ms of zeros */
    double *iq = (double *)calloc(2u * n, sizeof(double));
    bridge_outputs_t o;

    printf("[test_zero_signal]\n");
    if (!iq) { ASSERT(0, "allocate zero buffer"); return; }
    poison(&o);
    ASSERT(call_process(iq, n, fs, 0, &o) == BRIDGE_NO_FRAMES,
           "all-zero buffer at 100 Msps returns BRIDGE_NO_FRAMES");
    free(iq);
}

static void test_null_outputs_tolerated(void) {
    const double fs = 50e6;
    const uint32_t n = 100000;
    double *iq = (double *)calloc(2u * n, sizeof(double));
    int32_t rc;

    printf("[test_null_outputs_tolerated]\n");
    if (!iq) { ASSERT(0, "allocate buffer"); return; }
    rc = droneid_process(iq, n, fs, 0,
                         NULL, NULL, NULL, NULL, NULL, NULL, NULL,
                         NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);
    ASSERT(rc == BRIDGE_NO_FRAMES, "all output pointers may be NULL");
    free(iq);
}

int main(void) {
    printf("=== droneid_bridge unit tests ===\n");
    test_version();
    test_invalid_args();
    test_noise_returns_no_frames();
    test_zero_signal();
    test_null_outputs_tolerated();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
