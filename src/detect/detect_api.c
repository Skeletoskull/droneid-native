/**
 * detect_api.c — Public API implementation for droneid_detect library
 *
 * Implements detect_droneid(), detect_droneid_d(), detect_droneid_file(),
 * detect_free_results(), and detect_version().
 */

#ifndef DRONEID_DETECT_EXPORTS
#  define DRONEID_DETECT_EXPORTS
#endif
#include "../../include/droneid_detect.h"
#include "packet_detector.h"
#include <stdlib.h>
#include <string.h>
#include <complex.h>
#include <stdio.h>

#define DETECT_VERSION_MAJOR 1u
#define DETECT_VERSION_MINOR 0u
#define DETECT_VERSION_PATCH 0u

/* ── detect_droneid ──────────────────────────────────────────────────────── */

int32_t detect_droneid(const float *iq_interleaved, uint32_t num_samples,
                       double sample_rate, uint32_t flags,
                       detection_result_t **results_out,
                       uint32_t *num_results_out) {
    if (!iq_interleaved || num_samples == 0 || !results_out || !num_results_out)
        return DETECT_ERR_INVALID_ARG;

    *results_out     = NULL;
    *num_results_out = 0;

    /* DETECT_FLAG_CONJ: process a conjugated copy (negate Q) to undo a recorder
       spectral inversion. Otherwise reinterpret the input pairs in place. */
    if (flags & DETECT_FLAG_CONJ) {
        dsp_complex_t *cc = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * num_samples);
        if (!cc) return DETECT_ERR_ALLOC;
        for (uint32_t i = 0; i < num_samples; i++)
            cc[i] = CMPLX(iq_interleaved[2 * i], -iq_interleaved[2 * i + 1]);
        int32_t rc = packet_detect(cc, num_samples, sample_rate, flags,
                                   results_out, num_results_out);
        free(cc);
        return rc;
    }
    const dsp_complex_t *cx = (const dsp_complex_t *)iq_interleaved;
    return packet_detect(cx, num_samples, sample_rate, flags,
                         results_out, num_results_out);
}

/* ── detect_droneid_d ────────────────────────────────────────────────────── */

int32_t detect_droneid_d(const double *iq_interleaved, uint32_t num_samples,
                          double sample_rate, uint32_t flags,
                          detection_result_t **results_out,
                          uint32_t *num_results_out) {
    if (!iq_interleaved || num_samples == 0 || !results_out || !num_results_out)
        return DETECT_ERR_INVALID_ARG;

    *results_out     = NULL;
    *num_results_out = 0;

    /* Convert double IQ to float complex */
    dsp_complex_t *cx = (dsp_complex_t *)malloc(sizeof(dsp_complex_t) * num_samples);
    if (!cx) return DETECT_ERR_ALLOC;

    bool conj = (flags & DETECT_FLAG_CONJ) != 0;   /* negate Q to undo inversion */
    for (uint32_t i = 0; i < num_samples; i++) {
        float re = (float)iq_interleaved[2 * i];
        float im = (float)iq_interleaved[2 * i + 1];
        cx[i] = CMPLX(re, conj ? -im : im);
    }

    int32_t rc = packet_detect(cx, num_samples, sample_rate, flags,
                                results_out, num_results_out);
    free(cx);
    return rc;
}

/* ── detect_droneid_file ─────────────────────────────────────────────────── */

int32_t detect_droneid_file(const char *path, double sample_rate,
                             uint32_t flags,
                             detection_result_t **results_out,
                             uint32_t *num_results_out) {
    if (!path || !results_out || !num_results_out)
        return DETECT_ERR_INVALID_ARG;

    *results_out     = NULL;
    *num_results_out = 0;

    FILE *f = fopen(path, "rb");
    if (!f) return DETECT_ERR_INVALID_ARG;

    /* Get file size */
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (file_size <= 0) { fclose(f); return DETECT_ERR_INVALID_ARG; }

    /* File contains interleaved float32 I/Q pairs */
    uint32_t total_samples = (uint32_t)(file_size / (2 * sizeof(float)));
    uint32_t chunk_samples = (uint32_t)(0.5 * sample_rate); /* 500 ms */
    if (chunk_samples == 0) chunk_samples = total_samples;

    detection_result_t *all_results = NULL;
    uint32_t total = 0;
    int32_t final_rc = DETECT_STATUS_NO_FRAMES;

    float *buf = (float *)malloc(sizeof(float) * chunk_samples * 2);
    if (!buf) { fclose(f); return DETECT_ERR_ALLOC; }

    uint32_t samples_read = 0;
    while (samples_read < total_samples) {
        uint32_t to_read = chunk_samples;
        if (samples_read + to_read > total_samples)
            to_read = total_samples - samples_read;

        size_t got = fread(buf, sizeof(float) * 2, to_read, f);
        if (got == 0) break;

        detection_result_t *chunk_results = NULL;
        uint32_t chunk_count = 0;

        int32_t rc = detect_droneid(buf, (uint32_t)got, sample_rate, flags,
                                     &chunk_results, &chunk_count);
        if (rc == DETECT_OK && chunk_count > 0) {
            detection_result_t *tmp = (detection_result_t *)realloc(
                all_results, sizeof(detection_result_t) * (total + chunk_count));
            if (!tmp) {
                detect_free_results(chunk_results, chunk_count);
                detect_free_results(all_results, total);
                free(buf); fclose(f);
                return DETECT_ERR_ALLOC;
            }
            all_results = tmp;
            memcpy(all_results + total, chunk_results,
                   sizeof(detection_result_t) * chunk_count);
            free(chunk_results);
            total += chunk_count;
            final_rc = DETECT_OK;
        }
        samples_read += (uint32_t)got;
    }

    free(buf);
    fclose(f);

    if (total > 0) {
        *results_out     = all_results;
        *num_results_out = total;
        return DETECT_OK;
    }
    return final_rc;
}

/* ── detect_free_results ─────────────────────────────────────────────────── */

void detect_free_results(detection_result_t *results, uint32_t count) {
    if (!results) return;
    for (uint32_t i = 0; i < count; i++) {
        free(results[i].candidate_samples);
        results[i].candidate_samples = NULL;
    }
    free(results);
}

/* ── detect_version ──────────────────────────────────────────────────────── */

void detect_version(uint32_t *major, uint32_t *minor, uint32_t *patch) {
    if (major) *major = DETECT_VERSION_MAJOR;
    if (minor) *minor = DETECT_VERSION_MINOR;
    if (patch) *patch = DETECT_VERSION_PATCH;
}
