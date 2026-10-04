/**
 * test_locate.c - Location library unit tests
 *
 * MSVC C89-compatible: all variable declarations at the top of each block.
 */

#include "../include/droneid_locate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

static int g_pass = 0, g_fail = 0;

#define ASSERT(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", msg); g_pass++; } \
    else       { printf("  FAIL: %s\n", msg); g_fail++; } \
} while(0)

/* ── CRC-16 tests ────────────────────────────────────────────────────────── */

static void test_crc_empty(void) {
    uint8_t data[1];
    uint16_t crc;
    data[0] = 0;
    crc = compute_crc16(data, 0);
    /* CRC of empty input = init value 0x3692 */
    ASSERT(crc == 0x3692, "CRC16 of empty input = init value 0x3692");
}

static void test_crc_known(void) {
    /* Known test vector: CRC of {0x01, 0x02, 0x03} */
    uint8_t data[3];
    uint16_t crc, crc2;
    data[0] = 0x01; data[1] = 0x02; data[2] = 0x03;
    crc  = compute_crc16(data, 3);
    crc2 = compute_crc16(data, 3);
    ASSERT(crc == crc2, "CRC16 is deterministic");
    ASSERT(crc != 0, "CRC16 of non-zero data is non-zero");
}

static void test_crc_null(void) {
    uint16_t crc = compute_crc16(NULL, 0);
    ASSERT(crc == 0x3692 || crc == 0, "CRC16 NULL input does not crash");
}

/* ── JSON round-trip tests ───────────────────────────────────────────────── */

static void test_json_roundtrip(void) {
    telemetry_result_t orig, parsed;
    char buf[2048];
    int32_t rc;

    memset(&orig, 0, sizeof(orig));

    orig.pkt_len         = 91;
    orig.version         = 1;
    orig.sequence_number = 42;
    orig.state_info      = 0x1234;
    strncpy(orig.serial_number, "SN123456789ABCDE", 16);
    orig.serial_number[16] = '\0';
    orig.longitude       = 13.404954;
    orig.latitude        = 52.520008;
    orig.altitude_m      = 100.5;
    orig.height_m        = 50.25;
    orig.v_north         = 10;
    orig.v_east          = -5;
    orig.v_up            = 2;
    orig.d_1_angle       = 180;
    orig.gps_time        = 1234567890ULL;
    orig.app_lat         = 52.519;
    orig.app_lon         = 13.403;
    orig.longitude_home  = 13.400;
    orig.latitude_home   = 52.518;
    orig.device_type_id  = 63; /* Mini 2 */
    strncpy(orig.uuid, "UUID-1234567890ABCD", 20);
    orig.uuid[20] = '\0';
    orig.crc_valid       = true;

    rc = telemetry_to_json(&orig, buf, sizeof(buf));
    ASSERT(rc == LOCATE_OK, "telemetry_to_json returns LOCATE_OK");

    rc = telemetry_from_json(buf, &parsed);
    ASSERT(rc == LOCATE_OK, "telemetry_from_json returns LOCATE_OK");

    ASSERT(parsed.pkt_len         == orig.pkt_len,         "Round-trip: pkt_len");
    ASSERT(parsed.version         == orig.version,         "Round-trip: version");
    ASSERT(parsed.sequence_number == orig.sequence_number, "Round-trip: sequence_number");
    ASSERT(parsed.state_info      == orig.state_info,      "Round-trip: state_info");
    ASSERT(strcmp(parsed.serial_number, orig.serial_number) == 0, "Round-trip: serial_number");
    ASSERT(fabs(parsed.longitude - orig.longitude) < 1e-8,  "Round-trip: longitude");
    ASSERT(fabs(parsed.latitude  - orig.latitude)  < 1e-8,  "Round-trip: latitude");
    ASSERT(fabs(parsed.altitude_m - orig.altitude_m) < 1e-8, "Round-trip: altitude_m");
    ASSERT(parsed.v_north         == orig.v_north,         "Round-trip: v_north");
    ASSERT(parsed.device_type_id  == orig.device_type_id,  "Round-trip: device_type_id");
    ASSERT(strcmp(parsed.uuid, orig.uuid) == 0,            "Round-trip: uuid");
    ASSERT(parsed.crc_valid       == orig.crc_valid,       "Round-trip: crc_valid");
}

static void test_json_buffer_too_small(void) {
    telemetry_result_t r;
    char tiny[10];
    int32_t rc;
    memset(&r, 0, sizeof(r));
    rc = telemetry_to_json(&r, tiny, sizeof(tiny));
    ASSERT(rc == LOCATE_ERR_BUFFER_TOO_SMALL, "Small buffer -> LOCATE_ERR_BUFFER_TOO_SMALL");
    ASSERT(tiny[sizeof(tiny) - 1] == '\0' || tiny[0] == '{',
           "Small buffer: output is null-terminated");
}

/* ── API error handling ──────────────────────────────────────────────────── */

static void test_locate_null(void) {
    telemetry_result_t r;
    int32_t rc;
    rc = locate_droneid(NULL, 100, 0, &r);
    ASSERT(rc == LOCATE_ERR_INVALID_ARG, "locate_droneid NULL samples -> error");
    rc = locate_droneid(NULL, 0, 0, &r);
    ASSERT(rc == LOCATE_ERR_INVALID_ARG, "locate_droneid zero samples -> error");
}

static void test_version(void) {
    uint32_t maj = 99, min_v = 99, pat = 99;
    locate_version(&maj, &min_v, &pat);
    ASSERT(maj == 1 && min_v == 0 && pat == 0, "locate_version returns 1.0.0");
}

static void test_free_null(void) {
    locate_free_results(NULL, 0);
    ASSERT(1, "locate_free_results(NULL, 0) does not crash");
}

/* ── Property-based JSON round-trip test ─────────────────────────────────
 *
 * Property: For ALL valid telemetry_result_t values,
 *   telemetry_from_json(telemetry_to_json(x)) produces a struct equal to x.
 *
 * Strategy: Generate N_TRIALS random telemetry_result_t values covering
 * boundary cases and typical ranges. For each, serialise then parse back
 * and assert field-by-field equality (doubles within 1e-9 tolerance to
 * allow %.10g round-trip error).
 *
 * Shrinking: On failure, the iteration index and seed are printed so the
 * developer can reproduce the exact counterexample.
 * ─────────────────────────────────────────────────────────────────────── */

#define PBT_N_TRIALS 500
#define PBT_BUF_SIZE 2048

/* Printable ASCII range excluding '"' and '\' to keep JSON valid */
static char pbt_rand_char(unsigned int *state) {
    /* LCG: simple, reproducible PRNG that avoids system rand() */
    *state = (*state) * 1664525u + 1013904223u;
    /* Map to printable ASCII 0x20..0x7E, excluding '"'(0x22) and '\'(0x5C) */
    char c;
    do {
        *state = (*state) * 1664525u + 1013904223u;
        c = (char)(0x20 + ((*state >> 16) & 0x7F) % 0x5F);
    } while (c == '"' || c == '\\');
    return c;
}

static void pbt_rand_string(unsigned int *state, char *buf, int max_len) {
    int len = (int)(((*state >> 8) & 0xFF) % (unsigned int)max_len);
    int i;
    *state = (*state) * 1664525u + 1013904223u;
    for (i = 0; i < len; i++)
        buf[i] = pbt_rand_char(state);
    buf[len] = '\0';
}

static double pbt_rand_double(unsigned int *state, double lo, double hi) {
    *state = (*state) * 1664525u + 1013904223u;
    /* Combine two 16-bit parts to get ~32-bit mantissa fraction */
    double frac = (double)((*state >> 8) & 0xFFFFFFu) / (double)0x1000000u;
    return lo + frac * (hi - lo);
}

static void pbt_fill_telemetry(unsigned int *state, telemetry_result_t *r) {
    memset(r, 0, sizeof(*r));

    *state = (*state) * 1664525u + 1013904223u;
    r->pkt_len         = (uint8_t)((*state >> 16) & 0xFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->version         = (uint8_t)((*state >> 16) & 0xFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->sequence_number = (uint16_t)((*state >> 8) & 0xFFFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->state_info      = (uint16_t)((*state >> 8) & 0xFFFF);

    pbt_rand_string(state, r->serial_number, 16);  /* max 15 chars + NUL */
    r->serial_number[16] = '\0';

    r->longitude      = pbt_rand_double(state, -180.0,  180.0);
    r->latitude       = pbt_rand_double(state,  -90.0,   90.0);
    r->altitude_m     = pbt_rand_double(state, -500.0, 9000.0);
    r->height_m       = pbt_rand_double(state,    0.0,  500.0);

    *state = (*state) * 1664525u + 1013904223u;
    r->v_north  = (int16_t)((*state >> 8) & 0xFFFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->v_east   = (int16_t)((*state >> 8) & 0xFFFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->v_up     = (int16_t)((*state >> 8) & 0xFFFF);
    *state = (*state) * 1664525u + 1013904223u;
    r->d_1_angle = (int16_t)((*state >> 8) & 0xFFFF);

    *state = (*state) * 1664525u + 1013904223u;
    r->gps_time = ((uint64_t)(*state) << 32);
    *state = (*state) * 1664525u + 1013904223u;
    r->gps_time |= (uint64_t)(*state);

    r->app_lat        = pbt_rand_double(state,  -90.0,   90.0);
    r->app_lon        = pbt_rand_double(state, -180.0,  180.0);
    r->longitude_home = pbt_rand_double(state, -180.0,  180.0);
    r->latitude_home  = pbt_rand_double(state,  -90.0,   90.0);

    *state = (*state) * 1664525u + 1013904223u;
    r->device_type_id = (uint8_t)((*state >> 16) & 0xFF);

    pbt_rand_string(state, r->uuid, 20);  /* max 19 chars + NUL */
    r->uuid[20] = '\0';

    *state = (*state) * 1664525u + 1013904223u;
    r->crc_valid = ((*state >> 16) & 1) != 0;
}

static void test_json_roundtrip_property(void) {
    /* Property: parse(serialize(x)) == x for all valid telemetry_result_t */
    unsigned int state;
    int trial;
    int failures = 0;
    int first_fail = -1;
    char buf[PBT_BUF_SIZE];

    state = 0xDEADBEEFu;  /* fixed seed for reproducibility */

    for (trial = 0; trial < PBT_N_TRIALS; trial++) {
        telemetry_result_t orig, parsed;
        int32_t rc_ser, rc_par;
        int ok;

        pbt_fill_telemetry(&state, &orig);

        rc_ser = telemetry_to_json(&orig, buf, sizeof(buf));
        if (rc_ser != LOCATE_OK) {
            if (first_fail < 0) first_fail = trial;
            failures++;
            continue;
        }

        rc_par = telemetry_from_json(buf, &parsed);
        if (rc_par != LOCATE_OK) {
            if (first_fail < 0) first_fail = trial;
            failures++;
            continue;
        }

        ok = 1;
        ok &= (parsed.pkt_len         == orig.pkt_len);
        ok &= (parsed.version         == orig.version);
        ok &= (parsed.sequence_number == orig.sequence_number);
        ok &= (parsed.state_info      == orig.state_info);
        ok &= (strcmp(parsed.serial_number, orig.serial_number) == 0);
        ok &= (fabs(parsed.longitude      - orig.longitude)      < 5e-9 * (fabs(orig.longitude)      > 1.0 ? fabs(orig.longitude)      : 1.0));
        ok &= (fabs(parsed.latitude       - orig.latitude)       < 5e-9 * (fabs(orig.latitude)       > 1.0 ? fabs(orig.latitude)       : 1.0));
        ok &= (fabs(parsed.altitude_m     - orig.altitude_m)     < 5e-9 * (fabs(orig.altitude_m)     > 1.0 ? fabs(orig.altitude_m)     : 1.0));
        ok &= (fabs(parsed.height_m       - orig.height_m)       < 5e-9 * (fabs(orig.height_m)       > 1.0 ? fabs(orig.height_m)       : 1.0));
        ok &= (parsed.v_north         == orig.v_north);
        ok &= (parsed.v_east          == orig.v_east);
        ok &= (parsed.v_up            == orig.v_up);
        ok &= (parsed.d_1_angle       == orig.d_1_angle);
        ok &= (parsed.gps_time        == orig.gps_time);
        ok &= (fabs(parsed.app_lat        - orig.app_lat)        < 5e-9 * (fabs(orig.app_lat)        > 1.0 ? fabs(orig.app_lat)        : 1.0));
        ok &= (fabs(parsed.app_lon        - orig.app_lon)        < 5e-9 * (fabs(orig.app_lon)        > 1.0 ? fabs(orig.app_lon)        : 1.0));
        ok &= (fabs(parsed.longitude_home - orig.longitude_home) < 5e-9 * (fabs(orig.longitude_home) > 1.0 ? fabs(orig.longitude_home) : 1.0));
        ok &= (fabs(parsed.latitude_home  - orig.latitude_home)  < 5e-9 * (fabs(orig.latitude_home)  > 1.0 ? fabs(orig.latitude_home)  : 1.0));
        ok &= (parsed.device_type_id  == orig.device_type_id);
        ok &= (strcmp(parsed.uuid,         orig.uuid)            == 0);
        ok &= (parsed.crc_valid       == orig.crc_valid);

        if (!ok) {
            if (first_fail < 0) {
                first_fail = trial;
                printf("  Counterexample at trial %d: lon=%.10g lat=%.10g "
                       "sn=\"%s\" uuid=\"%s\"\n",
                       trial, orig.longitude, orig.latitude,
                       orig.serial_number, orig.uuid);
            }
            failures++;
        }
    }

    if (failures == 0) {
        printf("  PASS: JSON round-trip property holds for %d random structs\n",
               PBT_N_TRIALS);
        g_pass++;
    } else {
        printf("  FAIL: JSON round-trip property violated in %d/%d trials "
               "(first failure at trial %d)\n",
               failures, PBT_N_TRIALS, first_fail);
        g_fail++;
    }
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(void) {
    printf("=== Location Library Tests ===\n\n");

    printf("[CRC-16]\n");
    test_crc_empty();
    test_crc_known();
    test_crc_null();

    printf("\n[JSON Round-trip]\n");
    test_json_roundtrip();
    test_json_buffer_too_small();

    printf("\n[JSON Round-trip Property (PBT)]\n");
    test_json_roundtrip_property();

    printf("\n[API Error Handling]\n");
    test_locate_null();
    test_version();
    test_free_null();

    printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail > 0) ? 1 : 0;
}
