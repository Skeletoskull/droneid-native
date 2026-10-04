/**
 * droneid_validate.c — End-to-end validation of a recorded capture
 *
 * Runs a raw IQ capture through the native libraries (detect + locate) and
 * checks the decoded telemetry:
 *
 *   1. At least one DroneID frame is detected and decodes with a valid CRC.
 *   2. Every decoded frame passes field plausibility checks (coordinate
 *      ranges, string termination, JSON round-trip).
 *   3. If a reference file <iq_file>.ref.json exists (produced by
 *      telemetry_to_json), each decoded frame is compared against it
 *      field by field: numeric fields within 1e-6, strings exactly.
 *
 * Usage:
 *   droneid_validate <iq_file> <sample_rate_hz> [--legacy]
 *
 * Exit codes:
 *   0 — all checks passed
 *   1 — bad arguments or I/O error
 *   2 — one or more checks failed
 */

#include "droneid_detect.h"
#include "droneid_locate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── Tolerance for numeric field comparison ──────────────────────────────── */
#define NUMERIC_TOLERANCE 1e-6

/* ── Pass/fail counters ──────────────────────────────────────────────────── */
static int g_pass = 0;
static int g_fail = 0;
static int g_skip = 0;

#define CHECK(cond, msg) do { \
    if (cond) { printf("  PASS: %s\n", (msg)); g_pass++; } \
    else       { printf("  FAIL: %s\n", (msg)); g_fail++; } \
} while(0)

#define SKIP(msg) do { \
    printf("  SKIP: %s\n", (msg)); g_skip++; \
} while(0)

/* ── File existence check ────────────────────────────────────────────────── */
static int file_exists(const char *path) {
    FILE *f = fopen(path, "rb");
    if (f) { fclose(f); return 1; }
    return 0;
}

/* ── Coordinate plausibility check ──────────────────────────────────────── */
static int lat_plausible(double lat) { return lat >= -90.0 && lat <= 90.0; }
static int lon_plausible(double lon) { return lon >= -180.0 && lon <= 180.0; }

/* ── Reference JSON loader ────────────────────────────────────────── */
/* If a file <iq_path>.ref.json exists, load it and parse as telemetry_result_t.
   Returns 1 on success, 0 if the file does not exist or cannot be parsed.    */
static int load_reference(const char *iq_path, telemetry_result_t *ref_out) {
    char ref_path[1024];
    FILE *f;
    long sz;
    char *buf;
    int32_t rc;

    snprintf(ref_path, sizeof(ref_path), "%s.ref.json", iq_path);
    if (!file_exists(ref_path)) return 0;

    f = fopen(ref_path, "rb");
    if (!f) return 0;

    fseek(f, 0, SEEK_END);
    sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 65535) { fclose(f); return 0; }

    buf = (char *)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return 0; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        free(buf); fclose(f); return 0;
    }
    buf[sz] = '\0';
    fclose(f);

    rc = telemetry_from_json(buf, ref_out);
    free(buf);
    return (rc == LOCATE_OK) ? 1 : 0;
}

/* ── Compare a decoded frame against a reference struct ───────────── */
/* Returns 1 if all checks pass, 0 if any fail.                               */
static int compare_to_reference(const telemetry_result_t *native,
                                 const telemetry_result_t *ref,
                                 int frame_index) {
    int ok = 1;
    char msg[256];

#define CMP_NUM(field, label) do { \
    double diff = fabs((double)native->field - (double)ref->field); \
    snprintf(msg, sizeof(msg), "Frame %d: " label " within 1e-6 (diff=%.2e)", \
             frame_index, diff); \
    if (diff <= NUMERIC_TOLERANCE) { printf("  PASS: %s\n", msg); g_pass++; } \
    else { printf("  FAIL: %s (native=%.10g ref=%.10g)\n", msg, \
                  (double)native->field, (double)ref->field); g_fail++; ok = 0; } \
} while(0)

#define CMP_INT(field, label) do { \
    int match = ((native->field) == (ref->field)); \
    snprintf(msg, sizeof(msg), "Frame %d: " label " exact match", frame_index); \
    if (match) { printf("  PASS: %s\n", msg); g_pass++; } \
    else { printf("  FAIL: %s (native=%lld ref=%lld)\n", msg, \
                  (long long)(native->field), (long long)(ref->field)); \
           g_fail++; ok = 0; } \
} while(0)

#define CMP_STR(field, label) do { \
    int match = (strcmp(native->field, ref->field) == 0); \
    snprintf(msg, sizeof(msg), "Frame %d: " label " exact match", frame_index); \
    if (match) { printf("  PASS: %s\n", msg); g_pass++; } \
    else { printf("  FAIL: %s (native=\"%s\" ref=\"%s\")\n", msg, \
                  native->field, ref->field); g_fail++; ok = 0; } \
} while(0)

    CMP_INT(pkt_len,         "pkt_len");
    CMP_INT(version,         "version");
    CMP_INT(sequence_number, "sequence_number");
    CMP_INT(state_info,      "state_info");
    CMP_STR(serial_number,   "serial_number");
    CMP_NUM(longitude,       "longitude");
    CMP_NUM(latitude,        "latitude");
    CMP_NUM(altitude_m,      "altitude_m");
    CMP_NUM(height_m,        "height_m");
    CMP_INT(v_north,         "v_north");
    CMP_INT(v_east,          "v_east");
    CMP_INT(v_up,            "v_up");
    CMP_INT(d_1_angle,       "d_1_angle");
    CMP_INT(gps_time,        "gps_time");
    CMP_NUM(app_lat,         "app_lat");
    CMP_NUM(app_lon,         "app_lon");
    CMP_NUM(longitude_home,  "longitude_home");
    CMP_NUM(latitude_home,   "latitude_home");
    CMP_INT(device_type_id,  "device_type_id");
    CMP_STR(uuid,            "uuid");

    /* CRC validation result must match */
    {
        int match = (native->crc_valid == ref->crc_valid);
        snprintf(msg, sizeof(msg), "Frame %d: crc_valid matches", frame_index);
        if (match) { printf("  PASS: %s\n", msg); g_pass++; }
        else {
            printf("  FAIL: %s (native=%s ref=%s)\n", msg,
                   native->crc_valid ? "true" : "false",
                   ref->crc_valid    ? "true" : "false");
            g_fail++; ok = 0;
        }
    }

#undef CMP_NUM
#undef CMP_INT
#undef CMP_STR

    return ok;
}

/* ── Plausibility validation (no reference needed) ────────────────── */
static void validate_plausibility(const telemetry_result_t *tel, int frame_index) {
    char msg[256];

    snprintf(msg, sizeof(msg),
             "Frame %d: latitude in valid range [-90,90]", frame_index);
    CHECK(lat_plausible(tel->latitude), msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: longitude in valid range [-180,180]", frame_index);
    CHECK(lon_plausible(tel->longitude), msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: app_lat in valid range [-90,90]", frame_index);
    CHECK(lat_plausible(tel->app_lat), msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: app_lon in valid range [-180,180]", frame_index);
    CHECK(lon_plausible(tel->app_lon), msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: altitude_m in plausible range [-500,15000]", frame_index);
    CHECK(tel->altitude_m >= -500.0 && tel->altitude_m <= 15000.0, msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: serial_number is null-terminated", frame_index);
    CHECK(tel->serial_number[16] == '\0', msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: uuid is null-terminated", frame_index);
    CHECK(tel->uuid[20] == '\0', msg);
}

/* ── Validate CRC consistency using compute_crc16 ───────────────────────── */
static void validate_crc_consistency(const telemetry_result_t *tel, int frame_index) {
    /* The raw DUML bytes are not exposed, so verify internal consistency via
       an encode/decode round-trip of the decoded struct. */
    telemetry_result_t reparsed;
    char json_buf[4096];
    int32_t rc_ser, rc_par;
    char msg[256];

    rc_ser = telemetry_to_json(tel, json_buf, sizeof(json_buf));
    snprintf(msg, sizeof(msg),
             "Frame %d: telemetry_to_json succeeds", frame_index);
    CHECK(rc_ser == LOCATE_OK, msg);
    if (rc_ser != LOCATE_OK) return;

    rc_par = telemetry_from_json(json_buf, &reparsed);
    snprintf(msg, sizeof(msg),
             "Frame %d: telemetry_from_json (round-trip) succeeds", frame_index);
    CHECK(rc_par == LOCATE_OK, msg);
    if (rc_par != LOCATE_OK) return;

    snprintf(msg, sizeof(msg),
             "Frame %d: JSON round-trip: crc_valid preserved", frame_index);
    CHECK(reparsed.crc_valid == tel->crc_valid, msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: JSON round-trip: serial_number preserved", frame_index);
    CHECK(strcmp(reparsed.serial_number, tel->serial_number) == 0, msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: JSON round-trip: longitude within 1e-8", frame_index);
    CHECK(fabs(reparsed.longitude - tel->longitude) < 1e-8, msg);

    snprintf(msg, sizeof(msg),
             "Frame %d: JSON round-trip: latitude within 1e-8", frame_index);
    CHECK(fabs(reparsed.latitude - tel->latitude) < 1e-8, msg);
}

/* ── Process a single IQ file ────────────────────────────────────────────── */
static int process_file(const char *iq_path, double sample_rate, uint32_t flags) {
    detection_result_t *det_results = NULL;
    uint32_t n_det = 0;
    int32_t rc;
    uint32_t fi;
    int n_decoded = 0;
    int n_crc_ok = 0;
    int has_reference = 0;
    telemetry_result_t ref_tel;
    char msg[512];

    printf("===========================================================\n");
    printf("File: %s\n", iq_path);
    printf("Sample rate: %.0f Hz   Flags: 0x%x\n\n", sample_rate, (unsigned)flags);

    /* ── Check file exists ── */
    if (!file_exists(iq_path)) {
        fprintf(stderr, "  ERROR: cannot open %s\n", iq_path);
        return 1;
    }

    /* ── Try to load reference JSON ── */
    memset(&ref_tel, 0, sizeof(ref_tel));
    has_reference = load_reference(iq_path, &ref_tel);
    if (has_reference)
        printf("  INFO: reference JSON loaded.\n");
    else
        printf("  INFO: no reference JSON found; plausibility checks only.\n");
    printf("\n");

    /* ── Step 1: Detect frames ── */
    rc = detect_droneid_file(iq_path, sample_rate, flags, &det_results, &n_det);
    if (rc != DETECT_OK && rc != DETECT_STATUS_NO_FRAMES) {
        printf("  ERROR: detect_droneid_file returned %d\n", (int)rc);
        g_fail++;
        return 2;
    }

    snprintf(msg, sizeof(msg),
             "detect_droneid_file succeeds (rc=%d)", (int)rc);
    CHECK(rc == DETECT_OK || rc == DETECT_STATUS_NO_FRAMES, msg);

    printf("  Detected %u frame(s)\n\n", (unsigned)n_det);

    CHECK(n_det >= 1, "At least one DroneID frame detected");
    if (n_det == 0) {
        detect_free_results(det_results, n_det);
        return 0;
    }

    /* ── Step 2: Decode each frame ── */
    for (fi = 0; fi < n_det; fi++) {
        telemetry_result_t tel;
        char frame_hdr[128];

        memset(&tel, 0, sizeof(tel));

        snprintf(frame_hdr, sizeof(frame_hdr),
                 "--- Frame %u / %u ---", (unsigned)fi, (unsigned)n_det);
        printf("%s\n", frame_hdr);
        printf("  start_time_s = %.6f\n", det_results[fi].start_time_s);
        printf("  duration_s   = %.6f\n", det_results[fi].duration_s);
        printf("  cfo_hz       = %.1f\n", det_results[fi].cfo_hz);

        rc = locate_droneid(det_results[fi].candidate_samples,
                            det_results[fi].num_candidate_samples,
                            flags, &tel);
        if (rc != LOCATE_OK) {
            printf("  NOTE: locate_droneid returned %d", (int)rc);
            if (rc == LOCATE_ERR_ZC_NOT_FOUND)
                printf(" (LOCATE_ERR_ZC_NOT_FOUND — ZC synchronisation failed)");
            else if (rc == LOCATE_ERR_CRC_FAIL)
                printf(" (LOCATE_ERR_CRC_FAIL — no phase rotation gave valid CRC)");
            printf("\n  Skipping field-level checks for this frame.\n\n");
            SKIP("Decode failed — skipping field checks");
            continue;
        }

        n_decoded++;
        if (tel.crc_valid) n_crc_ok++;
        printf("  crc_valid    = %s\n", tel.crc_valid ? "true" : "false");
        printf("  serial       = \"%s\"\n", tel.serial_number);
        printf("  lat/lon      = %.6f / %.6f\n", tel.latitude, tel.longitude);
        printf("  altitude_m   = %.2f  height_m = %.2f\n",
               tel.altitude_m, tel.height_m);
        printf("\n");

        /* ── Plausibility checks (always run when decode succeeds) ── */
        validate_plausibility(&tel, (int)fi);

        /* ── CRC and JSON round-trip consistency ── */
        validate_crc_consistency(&tel, (int)fi);

        /* ── Field-by-field comparison against the reference ── */
        if (has_reference) {
            printf("  [Reference comparison]\n");
            compare_to_reference(&tel, &ref_tel, (int)fi);
        }
        printf("\n");
    }

    printf("  Decoded %d / %u frames (%d with valid CRC).\n\n",
           n_decoded, (unsigned)n_det, n_crc_ok);
    CHECK(n_crc_ok >= 1, "At least one frame decoded with a valid CRC");

    detect_free_results(det_results, n_det);
    return 0;
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main(int argc, char *argv[]) {
    const char *iq_path;
    double sample_rate;
    uint32_t flags = 0;
    int i;
    int rc;

    /* ── Argument parsing ── */
    if (argc < 3) {
        fprintf(stderr,
                "Usage: %s <iq_file> <sample_rate> [--legacy]\n\n"
                "  <iq_file>      Path to a raw IQ file (interleaved little-endian float32)\n"
                "  <sample_rate>  Sample rate in Hz (e.g. 50000000)\n"
                "  --legacy       Enable legacy drone support (Mavic Pro / Mavic 2 frame format)\n\n"
                "Example:\n"
                "  %s capture.bin 50000000\n\n"
                "Reference comparison:\n"
                "  If a file <iq_file>.ref.json exists it is parsed as a telemetry_result_t\n"
                "  JSON (produced by telemetry_to_json) and each decoded frame is compared\n"
                "  field-by-field: numeric fields within 1e-6, strings exactly.\n",
                argv[0], argv[0]);
        return 1;
    }

    iq_path     = argv[1];
    sample_rate = atof(argv[2]);

    if (sample_rate <= 0.0) {
        fprintf(stderr, "ERROR: sample_rate must be positive (got %s)\n", argv[2]);
        return 1;
    }

    for (i = 3; i < argc; i++) {
        if (strcmp(argv[i], "--legacy") == 0)
            flags |= LOCATE_FLAG_LEGACY;
    }

    /* ── Version banner ── */
    {
        uint32_t d_maj = 0, d_min = 0, d_pat = 0;
        uint32_t l_maj = 0, l_min = 0, l_pat = 0;
        detect_version(&d_maj, &d_min, &d_pat);
        locate_version(&l_maj, &l_min, &l_pat);
        printf("droneid_native validation tool\n");
        printf("  droneid_detect v%u.%u.%u\n", d_maj, d_min, d_pat);
        printf("  droneid_locate v%u.%u.%u\n\n", l_maj, l_min, l_pat);
    }

    /* ── Run validation ── */
    rc = process_file(iq_path, sample_rate, flags);

    /* ── Summary ── */
    printf("===========================================================\n");
    printf("Results: %d passed, %d failed, %d skipped\n",
           g_pass, g_fail, g_skip);

    if (rc != 0)     return rc;
    if (g_fail > 0)  return 2;
    return 0;
}
