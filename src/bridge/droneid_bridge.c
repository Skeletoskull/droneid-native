/**
 * droneid_bridge.c — LabVIEW-friendly bridge implementation
 */

#include "../../include/droneid_bridge.h"

/* Public detect/locate API headers */
#include "../../include/droneid_detect.h"
#include "../../include/droneid_locate.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define VERSION_MAJOR 1u
#define VERSION_MINOR 0u
#define VERSION_PATCH 0u

/* ── Helper: zero all outputs ────────────────────────────────────────────── */

static void zero_outputs(
    int32_t  *num_frames_out,
    int32_t  *crc_valid_out,
    char     *serial_out,
    double   *latitude_out,
    double   *longitude_out,
    double   *altitude_m_out,
    double   *height_m_out,
    double   *app_lat_out,
    double   *app_lon_out,
    int16_t  *v_north_out,
    int16_t  *v_east_out,
    uint16_t *sequence_number_out,
    uint8_t  *device_type_id_out,
    char     *json_out,
    uint32_t  json_buf_len)
{
    if (num_frames_out)      *num_frames_out      = 0;
    if (crc_valid_out)       *crc_valid_out        = -1;
    if (serial_out)          serial_out[0]         = '\0';
    if (latitude_out)        *latitude_out         = 0.0;
    if (longitude_out)       *longitude_out        = 0.0;
    if (altitude_m_out)      *altitude_m_out       = 0.0;
    if (height_m_out)        *height_m_out         = 0.0;
    if (app_lat_out)         *app_lat_out          = 0.0;
    if (app_lon_out)         *app_lon_out          = 0.0;
    if (v_north_out)         *v_north_out          = 0;
    if (v_east_out)          *v_east_out           = 0;
    if (sequence_number_out) *sequence_number_out  = 0;
    if (device_type_id_out)  *device_type_id_out   = 0;
    if (json_out && json_buf_len > 0) json_out[0]  = '\0';
}

/* ── Main bridge function ────────────────────────────────────────────────── */

int32_t droneid_process(
    const double  *iq_interleaved,
    uint32_t       num_samples,
    double         sample_rate,
    uint32_t       flags,
    int32_t       *num_frames_out,
    int32_t       *crc_valid_out,
    char          *serial_out,
    double        *latitude_out,
    double        *longitude_out,
    double        *altitude_m_out,
    double        *height_m_out,
    double        *app_lat_out,
    double        *app_lon_out,
    int16_t       *v_north_out,
    int16_t       *v_east_out,
    uint16_t      *sequence_number_out,
    uint8_t       *device_type_id_out,
    char          *json_out,
    uint32_t       json_buf_len)
{
    detection_result_t *det_results = NULL;
    uint32_t            n_det       = 0;
    int32_t             rc;
    uint32_t            fi;
    int32_t             n_decoded   = 0;

    /* Validate required inputs */
    if (!iq_interleaved || num_samples == 0)
        return BRIDGE_ERR_INVALID_ARG;

    /* Zero all outputs upfront */
    zero_outputs(num_frames_out, crc_valid_out, serial_out,
                 latitude_out, longitude_out, altitude_m_out, height_m_out,
                 app_lat_out, app_lon_out,
                 v_north_out, v_east_out,
                 sequence_number_out, device_type_id_out,
                 json_out, json_buf_len);

    /* ── Step 1: Detect DroneID frames ──────────────────────────────────── */
    rc = detect_droneid_d(iq_interleaved, num_samples, sample_rate, flags,
                          &det_results, &n_det);

    if (rc < 0) {
        return BRIDGE_ERR_DETECT;
    }

    if (n_det == 0 || !det_results) {
        detect_free_results(det_results, n_det);
        return BRIDGE_NO_FRAMES;
    }

    if (num_frames_out) *num_frames_out = (int32_t)n_det;

    /* ── Step 2: Decode each frame, report first successful one ─────────── */
    for (fi = 0; fi < n_det; fi++) {
        telemetry_result_t tel;
        memset(&tel, 0, sizeof(tel));

        rc = locate_droneid(det_results[fi].candidate_samples,
                            det_results[fi].num_candidate_samples,
                            flags, &tel);

        if (rc != LOCATE_OK) continue;

        n_decoded++;

        /* Populate outputs from first decoded frame */
        if (n_decoded == 1) {
            if (crc_valid_out)       *crc_valid_out        = tel.crc_valid ? 1 : 0;
            if (latitude_out)        *latitude_out         = tel.latitude;
            if (longitude_out)       *longitude_out        = tel.longitude;
            if (altitude_m_out)      *altitude_m_out       = tel.altitude_m;
            if (height_m_out)        *height_m_out         = tel.height_m;
            if (app_lat_out)         *app_lat_out          = tel.app_lat;
            if (app_lon_out)         *app_lon_out          = tel.app_lon;
            if (v_north_out)         *v_north_out          = tel.v_north;
            if (v_east_out)          *v_east_out           = tel.v_east;
            if (sequence_number_out) *sequence_number_out  = tel.sequence_number;
            if (device_type_id_out)  *device_type_id_out   = tel.device_type_id;

            if (serial_out) {
                /* Copy serial_number (always null-terminated, 17 bytes) */
                strncpy(serial_out, tel.serial_number, 16);
                serial_out[16] = '\0';
            }

            if (json_out && json_buf_len > 0) {
                telemetry_to_json(&tel, json_out, json_buf_len);
            }
        }
    }

    /* ── Step 3: Free detection results ─────────────────────────────────── */
    detect_free_results(det_results, n_det);

    if (n_decoded == 0) {
        /* Frames detected but none decoded (ZC not found, CRC fail, etc.) */
        if (crc_valid_out) *crc_valid_out = 0;
        return BRIDGE_NO_FRAMES;
    }

    return BRIDGE_OK;
}

/* ── Version ─────────────────────────────────────────────────────────────── */

void droneid_bridge_version(uint32_t *major, uint32_t *minor, uint32_t *patch) {
    if (major) *major = VERSION_MAJOR;
    if (minor) *minor = VERSION_MINOR;
    if (patch) *patch = VERSION_PATCH;
}
